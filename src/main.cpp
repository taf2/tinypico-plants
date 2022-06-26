#define SEESAW_I2C_DEBUG
#include <TinyPICO.h>
#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_seesaw.h" // so we can use the soil capacitive touch sensor to measure the moisture of the soil for the plant

const int MinPumpVolts = 1.8;
const int MaxPumpCycles = 1;
const int MaxPumpDuration = 30;
float tempC;
uint16_t capread;

const unsigned long uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
const unsigned long TIME_TO_SLEEP  = 60*60;    // Time ESP32 will go to sleep (in seconds) e.g. 1 hour
RTC_DATA_ATTR unsigned long wakeCount = 0; // we can only deep sleep 1 hour, so wake up every hour... increment wakeCount if it reaches 8 it's time to water our plant

const int CAM_WAKE = 4;
const int ChargeStatus = 27; // voltage sensor between 2 2k resistors to sense once the voltage is 2.7 then we are fully charged with the two super caps at 5.4 volts
const int ChargePin_Pchannel = 19;
const int ChargePin_Nchannel = 14;
const int ChargePrimePin_Pchannel = 5; // we charge this capacitor first then we use this capacitor to feed our super caps
const int ChargeOnPin = 33;
const int ChargeDone = 32;
const int ChargeStatusBase = 25; // monitor the voltage on our initial charge capacitors 2 500Ohm resistors when we see 2.7 we have 4.6 volts
const int MotorOn_Nchannel = 18;

const int FullWarmUpReading = 2.7;
const int MaxWarmUpReading = 2.5;
const int MinWarmUpReading = 1.9;

bool ChargeMore = false;
int iterations = 0;

Adafruit_seesaw soilSensor;
void doCam();

void warmup(long full=false, int maxIterations=-1) {
  float reading = analogRead(ChargeStatus) / 1000.0;
  float baseReading = analogRead(ChargeStatusBase) / 1000.0;
  const int warmupTo = full ? FullWarmUpReading : MaxWarmUpReading;
  int interations = 0;
  Serial.printf("                                                   \r");
  while (baseReading < warmupTo) {
    digitalWrite(ChargePrimePin_Pchannel, LOW); // prime the initial capacitor bank
    digitalWrite(ChargeOnPin, LOW);
    delay(0.1);
    digitalWrite(ChargeOnPin, HIGH);
    baseReading = analogRead(ChargeStatusBase) / 1000.0;
    Serial.printf("warming up super cap at %f and base super cap: %f\r", reading, baseReading);
    digitalWrite(ChargePrimePin_Pchannel, HIGH);
    if (maxIterations > 0) {
      if (iterations > maxIterations) {
        break;
      }
    }
    interations++;
  }
}

void chargeup(int maxIterations=-1) {
  float reading = analogRead(ChargeStatus) / 1000.0;
  float baseReading = analogRead(ChargeStatusBase) / 1000.0;
  digitalWrite(ChargeDone, LOW);
  digitalWrite(ChargeOnPin, HIGH);

  digitalWrite(ChargePin_Nchannel, HIGH);
  delay(1); // give time to allow the boost to charge up 1000 uF cap
  digitalWrite(ChargePin_Pchannel, LOW);

  int iterations = 0;

  while ((baseReading > MinWarmUpReading && reading < 2.974)) {
    delay(0.1);
    baseReading = analogRead(ChargeStatusBase) / 1000.0;
    reading = analogRead(ChargeStatus) / 1000.0;
    Serial.printf("charing up super cap at %f and base super cap: %f\r", reading, baseReading);
    iterations++;
    if (maxIterations > 0) {
      if (iterations > maxIterations) {
        break;
      }
    }
  }
  digitalWrite(ChargePin_Nchannel, LOW);
  digitalWrite(ChargePin_Pchannel, HIGH);
}
enum WakeUpMode { WakeTimer, WakeMotion, WakeOther, WakeBootOrError };
static WakeUpMode wakeup_reason();

void setup() {
  Serial.begin(115200);
  pinMode(ChargePin_Pchannel, OUTPUT);
  pinMode(ChargePin_Nchannel, OUTPUT);
  pinMode(ChargePrimePin_Pchannel, OUTPUT);
  pinMode(ChargeOnPin, OUTPUT);
  pinMode(ChargeStatus, INPUT);
  pinMode(ChargeStatusBase, INPUT);
  pinMode(ChargeDone, OUTPUT);
  pinMode(MotorOn_Nchannel, OUTPUT);
  pinMode(CAM_WAKE, OUTPUT); // signal to take a picture

  digitalWrite(CAM_WAKE, LOW);
  digitalWrite(ChargeDone, LOW);
  digitalWrite(ChargePin_Nchannel, LOW);
  digitalWrite(ChargePin_Pchannel, HIGH);
  digitalWrite(ChargePrimePin_Pchannel, HIGH);
  digitalWrite(MotorOn_Nchannel, LOW); // pull low to turn it off
  delay(1000);

  if (!soilSensor.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(soilSensor.getVersion(), HEX);
  }

  tempC = soilSensor.getTemp();
  capread = soilSensor.touchRead(0);
  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(capread);
  if (capread > 800) {
    wakeCount = 0;
    Serial.println("looks pretty moist in there let's wait until it's dry");
    // wake up on timer
    delay(1000);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    return;
  }

  WakeUpMode wake = wakeup_reason();
  if (wake == WakeTimer) {
    wakeCount ++;
    if (wakeCount >= 8) {
      wakeCount = 0;
      warmup();
    } else {
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  } else if (wake == WakeBootOrError) {
    Serial.println("wake on error or first boot/power");
    wakeCount  = 0; // like a reset
    warmup();
  } else {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}

int PumpCount = 0;

void doCam() {
  Serial.println("Capture a Picture");
  digitalWrite(CAM_WAKE, HIGH); // wakes up esp32cam to send picture to us later
  delay(1000);
  digitalWrite(CAM_WAKE, LOW);
  delay(35000); // wait 35 seconds for camera to do it's work
}

// we can run the pump for about MaxPumpDuration seconds which is not enough water so we need to pump a few times
int doPump() {

  warmup();
  digitalWrite(ChargeOnPin, LOW);
  digitalWrite(ChargeDone, HIGH);

  Serial.println("shut down boost");
  // disconnect from main disables boost charger
  digitalWrite(ChargePin_Nchannel, LOW);

  Serial.println("shut down pchannel");
  // disables pchannel mosfet
  digitalWrite(ChargePin_Pchannel, HIGH);
  digitalWrite(ChargePrimePin_Pchannel, HIGH);

  Serial.println("enable pump");
  // at this point motor should only be able to draw current from our super capacitors
  //
  digitalWrite(MotorOn_Nchannel, HIGH); // enable the pump

  Serial.printf("pump for %d seconds", MaxPumpDuration);
  uint64_t start_time = millis();

  for (int i = 0; i < MaxPumpDuration; ++i) {
    uint64_t start_charge = millis();
    warmup(false, 5000); // 0.5 * 1000 == 500 
    chargeup(1250); // 0.5 * 1000 == 500 
    digitalWrite(ChargeDone, HIGH);
    digitalWrite(MotorOn_Nchannel, HIGH); // enable the pump
    float reading = analogRead(ChargeStatus) / 1000.0;
    Serial.printf("\npumped %d of %d seconds with %f\n", i+1, MaxPumpDuration, reading);
    if (reading < MinPumpVolts) {
      Serial.printf("stop pump reading is less than 2.3 at %f\n", reading);
      break; // we can get more later
    }
    uint64_t charge_time = millis();
    uint64_t interval = charge_time - start_charge;
    uint64_t total_time = charge_time - start_time;
    Serial.printf("time of interval: %ld time total: %ld\n", interval, total_time);

    if (total_time >= (MaxPumpDuration*1000)) {
      break;
    }
    if (interval < 1000) {
      uint64_t extended_time = 1000 - interval;
      Serial.printf("continue waiting: %ld\n", extended_time);
      delay(extended_time); // we want to pump for 20 seconds
    }
  }
  digitalWrite(MotorOn_Nchannel, LOW); // turn the pump off
  return PumpCount ++;
}

void doCharge() {
  float reading = analogRead(ChargeStatus) / 1000.0;
  ChargeMore = (reading < 2.974);
  if (ChargeMore) {
    digitalWrite(MotorOn_Nchannel, LOW); // ensure  the pump is OFF
    digitalWrite(ChargePin_Nchannel, LOW);

    digitalWrite(ChargePrimePin_Pchannel, LOW); // prime the initial capacitor bank
    warmup();
    //digitalWrite(ChargePrimePin_Pchannel, HIGH); // turn it off so we're only drawing from the cap for a short burst
    chargeup();

  } else {
    digitalWrite(MotorOn_Nchannel, LOW); // ensure  the pump is OFF
    digitalWrite(ChargeDone, HIGH);
    digitalWrite(ChargeOnPin, LOW);
    digitalWrite(ChargePin_Nchannel, LOW);
    digitalWrite(ChargePin_Pchannel, HIGH);
    digitalWrite(ChargePrimePin_Pchannel, HIGH);
    delay(1000);
    if (doPump() == MaxPumpCycles) { // once we have done MaxPumpCycles pumps we sleep for 8 hours
      doCam();

      //tempC = soilSensor.getTemp();
      //capread = soilSensor.touchRead(0);
      Serial.printf("going to sleep we finished %d pumps with: %f C and %ld\n", MaxPumpCycles+1, tempC, capread);
      // wake up on timer
      delay(1000);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    } else {
      Serial.printf("do more pumps %d of %d\n", PumpCount, MaxPumpCycles);
    }
  }
}

void loop() {
  doCharge();
}

WakeUpMode wakeup_reason() {
  esp_sleep_wakeup_cause_t wr;

  wr = esp_sleep_get_wakeup_cause();

  switch (wr) {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    return WakeMotion;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    return WakeOther;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    return WakeTimer;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    return WakeOther;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    return WakeOther;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n",wr);
    return WakeBootOrError;
  }
  return WakeOther;
}
