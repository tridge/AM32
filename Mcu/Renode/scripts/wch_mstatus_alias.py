# CSR 0x800: WCH's machine-mode alias of mstatus. core_riscv.h's
# __enable_irq()/__disable_irq() write 0x1888/0x1800 here instead of
# using csrsi/csrci on mstatus itself. Forward both directions.
if request.IsWrite:
    cpu.MSTATUS = request.Value
elif request.IsRead:
    request.Value = cpu.MSTATUS.RawValue
