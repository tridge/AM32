# Skip AM32's busy-wait delays instead of emulating every poll of the
# microsecond timer.
#
# The delay loop spins on UTILITY_TIMER->CNT, costing one native to
# managed transition per read. playStartupTune() spends 600ms in it and
# that dominates boot: with the tune running, 0.7s of virtual time costs
# 96s of wall clock against about 10s for the main loop. Renode can jump
# virtual time forward without executing instructions (SkipTime), which
# is Antmicro's suggested treatment for busy-wait loops.
#
# HOOK delayMillis, NOT delayMicros. At -O3 gcc inlines delayMicros into
# delayMillis - the out-of-line delayMicros symbol still exists, so a
# hook on it attaches without complaint and then never fires, because
# the tune reaches the loop through delayMillis.
#
# GATED ON INTERRUPTS BEING MASKED. We only skip when the firmware has
# already disabled interrupts, which is the case the tune runs in
# (Src/sounds.c:120 does __disable_irq() around its delays). That makes
# the skip provably free of side effects: with interrupts masked there
# is nothing that could have been delivered during the wait, so it does
# not matter whether Renode would have delivered it. Delays called with
# interrupts live are emulated normally.
#
# Loaded with:
#   cpu AddSymbolHook "delayMillis" "execfile('.../skip_delays.py')"

from Antmicro.Renode.Time import TimeInterval

if hasattr(cpu, 'GetPrimask'):
    # Cortex-M: PRIMASK masks; r0 is the millis argument, LR the return
    masked = cpu.GetPrimask(False) != 0
    ms = cpu.GetRegister(0).RawValue
    ret = cpu.LR
else:
    # RISC-V: masked when mstatus.MIE (bit 3) is clear; the argument is
    # a0, the return address ra
    masked = (cpu.MSTATUS.RawValue & 0x8) == 0
    ms = cpu.GetRegisterUnsafe(10).RawValue
    ret = cpu.GetRegisterUnsafe(1).RawValue

if masked:
    if ms > 0:
        cpu.SkipTime(TimeInterval.FromMilliseconds(ms))
    cpu.PC = ret
