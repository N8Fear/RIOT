APPLICATION = test_pthread_cleanup
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

USEMODULE += pthread

include $(RIOTBASE)/Makefile.include
