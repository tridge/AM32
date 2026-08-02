/*
  Lets Renode drive the SITL motor model instead of forking it.

  Mcu/SITL/sim/motor.c is the calibrated physics, refit against real
  hardware more than once, so a C# transliteration would diverge on the
  next recalibration and leave two models with no ground truth. It is
  compiled here unmodified and reached over P/Invoke.

  motor.c needs three things from its host: the per-phase bridge mode,
  the TIM1 output state, and somewhere to put the comparator result.
  Under the SITL those come from the fake peripherals in Mcu/SITL/Src;
  here they come from the real emulated registers, which is the whole
  point - the register code under Mcu/f051/Src actually runs.

  The rest of what motor.c references is firmware globals used only by
  its logging helpers. They are defined here and stay zero: the firmware
  runs inside the emulator, not in this process, so nothing can update
  them. Physics does not read them. If the physics logs are ever wanted
  from Renode, push them in from emulated SRAM rather than reviving the
  SITL wiring.
*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sitl.h"
#include "sitl_config.h"
#include "eeprom.h"
#include "motor.h"

/* ---- state motor.c reads, pushed in from the emulated registers ---- */

volatile uint8_t sitl_phase_mode[3];
volatile uint8_t sitl_comp_out;
volatile uint8_t sitl_comp_phase = 2;
sitl_exti_t sitl_exti;
uint32_t current_EXTI_LINE;

static struct {
    uint32_t arr;
    uint32_t ccr[3];
    uint32_t dead_ns;
    uint64_t tick_ps; /* one timer tick */
    uint64_t cnt_ps;  /* position within the PWM period */
    uint64_t now_ns;
    uint64_t last_ns;
    bool started;
} st;

uint64_t sitl_time_ns(void) { return st.now_ns; }

/*
  the counter is advanced in picoseconds per sub-step rather than
  recomputed from an epoch, so a PSC or ARR change mid-run cannot make
  the phase jump
*/
bool sitl_tim1_pwm_out(int chan, uint64_t now_ns)
{
    (void)now_ns;
    if (chan < 0 || chan > 2 || st.tick_ps == 0) {
        return false;
    }
    return (st.cnt_ps / st.tick_ps) < st.ccr[chan];
}

uint32_t sitl_tim1_dead_time_ns(void) { return st.dead_ns; }

/* Renode's EXTI model raises the comparator interrupt from the pin
   change, so motor.c's request is a no-op here */
void sitl_irq_pend(int irq) { (void)irq; }

/* ---- firmware globals; logging only, see the header comment ---- */

volatile uint16_t duty_cycle;
uint16_t duty_cycle_maximum;
volatile uint32_t commutation_interval;
volatile uint32_t average_interval;
uint32_t last_average_interval;
uint32_t desync_happened;
volatile uint32_t zero_crosses;
int e_com_time;
uint16_t input;
volatile char armed;
char step;
uint8_t bemf_timeout_happened;
uint8_t running;
char old_routine;
volatile uint16_t newinput;
uint16_t adjusted_input;
EEprom_t eepromBuffer;

void sitl_can_stats(uint32_t stats[4]) { memset(stats, 0, 4 * sizeof(uint32_t)); }

/* ---- API called from the Renode peripheral ---- */

int am32sim_init(const char* config_path)
{
    char arg0[] = "am32sim";
    char opt[] = "--config";
    char* argv[3];
    int argc = 1;

    argv[0] = arg0;
    if (config_path != NULL && config_path[0] != '\0') {
        argv[1] = opt;
        argv[2] = (char*)config_path;
        argc = 3;
    }
    sitl_config_init(argc, argv);
    motor_init();
    memset(&st, 0, sizeof(st));
    st.tick_ps = 20833; /* 48MHz, PSC=0, until Renode says otherwise */
    return 0;
}

void am32sim_set_bridge(int a, int b, int c)
{
    sitl_phase_mode[0] = (uint8_t)a;
    sitl_phase_mode[1] = (uint8_t)b;
    sitl_phase_mode[2] = (uint8_t)c;
}

void am32sim_set_tim1(uint32_t arr, uint32_t ccr_a, uint32_t ccr_b,
                      uint32_t ccr_c, uint32_t tick_ps, uint32_t dead_ns)
{
    st.arr = arr;
    st.ccr[0] = ccr_a;
    st.ccr[1] = ccr_b;
    st.ccr[2] = ccr_c;
    st.tick_ps = tick_ps ? tick_ps : 1;
    st.dead_ns = dead_ns;
}

void am32sim_set_comp_phase(int phase)
{
    if (phase >= 0 && phase <= 2) {
        sitl_comp_phase = (uint8_t)phase;
    }
}

/*
  advance the physics to now_ns in sitl_cfg.sim.physics_dt_ns sub-steps,
  keeping the PWM counter in step so an edge inside the batch lands at
  the right sub-step rather than at the batch boundary. Returns the
  comparator output, which motor.c has updated. "driven" says whether the
  bridge is energised; a stationary undriven motor is skipped entirely.
*/
int am32sim_advance(uint64_t now_ns, int driven)
{
    const uint32_t dt = sitl_cfg.sim.physics_dt_ns ? sitl_cfg.sim.physics_dt_ns : 500;
    const uint64_t period_ps = (uint64_t)(st.arr + 1) * st.tick_ps;

    if (!st.started) {
        st.started = true;
        st.last_ns = now_ns;
        st.now_ns = now_ns;
        return sitl_comp_out;
    }
    /* nothing driving and nothing turning: no state can change, so skip
       the integration rather than grind through a motor that cannot
       move. This is most of boot. */
    if (!driven) {
        double th, om, i[3];
        motor_get_state(&th, &om, i);
        if (fabs(om) < 1e-3) {
            st.last_ns = now_ns;
            st.now_ns = now_ns;
            return sitl_comp_out;
        }
    }
    /* a long gap means the emulator jumped; resynchronise rather than
       grinding through millions of sub-steps */
    if (now_ns > st.last_ns + 100000000ULL) {
        st.last_ns = now_ns - dt;
    }
    while (st.last_ns + dt <= now_ns) {
        st.last_ns += dt;
        st.now_ns = st.last_ns;
        if (period_ps) {
            st.cnt_ps = (st.cnt_ps + (uint64_t)dt * 1000) % period_ps;
        }
        motor_step(st.now_ns, dt);
    }
    st.now_ns = now_ns;
    return sitl_comp_out;
}

/* bus voltage, bus current and temperature, which the ADC model turns
   into raw counts for the firmware to read over DMA. Same source the
   SITL's Mcu/SITL/Src/ADC.c uses. */
void am32sim_get_sensors(double* volts, double* amps, double* degrees)
{
    sitl_sensors_t s;
    sitl_sensors_read(&s);
    if (volts) {
        *volts = s.bus_voltage;
    }
    if (amps) {
        *amps = s.bus_current;
    }
    if (degrees) {
        *degrees = s.temperature_c;
    }
}

/* phase currents, amps. These are motor truth; the firmware sees only
   what the ADC model gives it. */
void am32sim_get_currents(double i[3])
{
    double th, om;
    motor_get_state(&th, &om, i);
}

void am32sim_get_state(double* omega, double* theta, double* rpm)
{
    double th, om, i[3];
    motor_get_state(&th, &om, i);
    if (omega) {
        *omega = om;
    }
    if (theta) {
        *theta = th;
    }
    if (rpm) {
        /* mechanical rpm */
        *rpm = om * 60.0 / (2.0 * 3.14159265358979);
    }
}
