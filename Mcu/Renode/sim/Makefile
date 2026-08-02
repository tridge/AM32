# builds the SITL motor model as a shared library for the Renode bridge.
# motor.c and sitl_config.c are used unmodified; am32sim_shim.c supplies
# what the SITL's fake peripherals would have.

ROOT := ../../..
SITL := $(ROOT)/Mcu/SITL
OUT  := $(ROOT)/obj

SRCS := am32sim_shim.c $(SITL)/sim/motor.c $(SITL)/sim/sitl_config.c $(SITL)/sim/jsmn.c
INCS := -I$(SITL)/Inc -I$(SITL)/sim -I$(ROOT)/Inc -I$(ROOT)/Mcu/f051/Inc

CFLAGS := -O2 -g -fPIC -Wall -Wno-unused-parameter
LDFLAGS := -shared -Wl,-soname,libam32sim.so -lm -lpthread

all: $(OUT)/libam32sim.so

$(OUT)/libam32sim.so: $(SRCS)
	@mkdir -p $(OUT)
	$(CC) $(CFLAGS) $(INCS) $(SRCS) -o $@ $(LDFLAGS)

clean:
	rm -f $(OUT)/libam32sim.so

.PHONY: all clean
