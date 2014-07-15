APPLICATION = test_struct_tm_utility
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: time.h missing from avr-libc

DISABLE_MODULE += auto_init

USEMODULE += shell
USEMODULE += posix
USEMODULE += timex

# The MSP-430 toolchain lacks sscanf:
BOARD_BLACKLIST := chronos msb-430 msb-430h telosb wsn430-v1_3b wsn430-v1_4 z1

# Too little RAM
BOARD_BLACKLIST += redbee-econotag

include $(RIOTBASE)/Makefile.include
