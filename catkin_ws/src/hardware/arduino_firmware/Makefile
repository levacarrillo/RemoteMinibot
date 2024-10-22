ARDUINO_DIR = /usr/share/arduino
ARDUINO_PORT = /dev/ttyACM0


CURRENT_DIR := $(shell dirname $(shell pwd))
$(info CURRENT_DIR is $(CURRENT_DIR))
USER_LIB_PATH += $(CURRENT_DIR)/include/hardware
ARDUINO_LIBS += ros_lib
ARDUINO_LIBS += sensors
ARDUINO_LIBS += encoders
ARDUINO_LIBS += motors
ARDUINO_LIBS += pid

BOARD_TAG = mega2560

include /usr/share/arduino/Arduino.mk
