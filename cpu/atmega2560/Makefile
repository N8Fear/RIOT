
# define the module that is build
MODULE = cpu

# add a list of subdirectories, that should also be build
DIRS = periph

.PHONY: all $(ATMEGA_COMMON)

# when building the CPU module make sure, the std peripheral lib will also be build
all: $(ATMEGA_COMMON) $(BINDIR)$(MODULE).a
	@for i in $(DIRS) ; do $(MAKE) -C $$i ; done ;

$(ATMEGA_COMMON):
	@$(MAKE) -C $(ATMEGA_COMMON)

include $(RIOTBASE)/Makefile.base

clean::
	@for i in $(DIRS) ; do $(MAKE) -C $$i clean ; done ;
	@$(MAKE) -C $(ATMEGA_COMMON) clean
