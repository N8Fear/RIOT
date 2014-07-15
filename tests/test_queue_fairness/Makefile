APPLICATION = test_queue_fairness
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

BOARD_INSUFFICIENT_RAM := mbed_lpc1768

USEMODULE += vtimer

DISABLE_MODULE += auto_init

include $(RIOTBASE)/Makefile.include
