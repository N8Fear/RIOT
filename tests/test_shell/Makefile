APPLICATION = test_shell
include ../Makefile.tests_common

BOARD_BLACKLIST := arduino-mega2560
# arduino-mega2560: malloc.h missing from avr-libc

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += posix

DISABLE_MODULE += auto_init

include $(RIOTBASE)/Makefile.include
