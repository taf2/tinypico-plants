ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: detector
upload: detector
	pio run -j 2 --target upload && sleep 1;  pio device monitor

detector: src/main.cpp
	pio run

monitor:
	pio device monitor

clean:
	pio run --target clean
