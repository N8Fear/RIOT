APPLICATION = test_pthread_rwlock
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

USEMODULE += pthread
USEMODULE += vtimer
USEMODULE += random

DISABLE_MODULE += auto_init

CFLAGS += -DNATIVE_AUTO_EXIT

BOARD_INSUFFICIENT_RAM += chronos mbed_lpc1768 msb-430 msb-430h

include $(RIOTBASE)/Makefile.include
