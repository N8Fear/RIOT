APPLICATION = test_condition_variable
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

USEMODULE += posix
USEMODULE += pthread
USEMODULE += vtimer

CFLAGS += -DNATIVE_AUTO_EXIT

include $(RIOTBASE)/Makefile.include
