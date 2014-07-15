APPLICATION = test_vtimer_msg_diff
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

BOARD_INSUFFICIENT_RAM := mbed_lpc1768

USEMODULE += vtimer

include $(RIOTBASE)/Makefile.include
