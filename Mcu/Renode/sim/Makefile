# builds the SITL motor model as a shared library for the Renode bridge.
# motor.c and sitl_config.c are used unmodified; am32sim_shim.c supplies
# what the SITL's fake peripherals would have.

ROOT := ../../..
SITL := $(ROOT)/Mcu/SITL
OUT  := $(ROOT)/obj
OBJDIR := $(OUT)/renode_sim

SRCS := am32sim_shim.c $(SITL)/sim/motor.c $(SITL)/sim/sitl_config.c $(SITL)/sim/jsmn.c
OBJS := $(addprefix $(OBJDIR)/,$(notdir $(SRCS:.c=.o)))
INCS := -I$(SITL)/Inc -I$(SITL)/sim -I$(ROOT)/Inc -I$(ROOT)/Mcu/f051/Inc

CFLAGS := -O2 -g -fPIC -Wall -Wno-unused-parameter
LDFLAGS := -shared -Wl,-soname,libam32sim.so -lm -lpthread

vpath %.c . $(SITL)/sim

all: $(OUT)/libam32sim.so

# per object with -MMD, so editing motor.h or sitl.h rebuilds the
# library. Compiling the sources in one command cannot express header
# dependencies, and the test runner would then happily test a stale
# binary after a header-only change.
$(OBJDIR)/%.o: %.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) $(INCS) -MMD -MP -c $< -o $@

$(OUT)/libam32sim.so: $(OBJS)
	@mkdir -p $(OUT)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

clean:
	rm -rf $(OBJDIR)
	rm -f $(OUT)/libam32sim.so

-include $(OBJS:.o=.d)

.PHONY: all clean
