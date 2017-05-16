ARDUINO_LIBS = SPI EEPROM
BOARD_TAG    = uno
MONITOR_PORT = /dev/ttyACM*
# USER_LIB_PATH := $(realpath ../../libraries)
include /usr/share/arduino/Arduino.mk
