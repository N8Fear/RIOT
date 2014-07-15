APPLICATION = test_posix_semaphore
include ../Makefile.tests_common

BOARD_INSUFFICIENT_RAM := msb-430 msb-430h mbed_lpc1768 redbee-econotag chronos

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

USEMODULE += posix

DISABLE_MODULE += auto_init

include $(RIOTBASE)/Makefile.include
