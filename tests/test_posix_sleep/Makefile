APPLICATION = test_posix_sleep
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

USEMODULE += posix

DISABLE_MODULE += auto_init

include $(RIOTBASE)/Makefile.include
