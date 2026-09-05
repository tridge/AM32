# AM32 SITL (software in the loop)

Runs the AM32 firmware as a native Linux executable against a simulation
of the motor, bridge and battery, with DroneCAN input/output over
multicast UDP. This allows testing of the firmware logic (startup,
commutation, DroneCAN protocol, parameters) without ESC hardware.

## Building

```
make AM32_SITL_CAN
```

produces `obj/AM32_AM32_SITL_CAN_<version>.elf`, a normal Linux
executable.

### Sanitizers and valgrind

`SITL_SANITIZE=address` builds with AddressSanitizer (heap/stack
overflow and use-after-free), `SITL_SANITIZE=undefined` with UBSan, and
both can be combined (`address,undefined`):

```
make AM32_SITL_CAN SITL_SANITIZE=address
```

Both run at close to full speed, so it is practical to leave a
sanitizer on while driving the sim or running the whole test suite.
`./asan.sh` and `./ubsan.sh` build the respective sanitizer if stale
and run it, writing any reports to `asan.<pid>` / `ubsan.<pid>` files
so they are not buried in the SITL's verbose output. ASan's leak
detector only reports on a clean exit, so under the (kill-terminated)
test harness ASan catches overflows and use-after-free during the run
rather than leaks at exit. The whole test suite passes under each, and
both run as their own CI jobs.

`./valgrind.sh` runs the normal build under valgrind memcheck instead
(slower - about a tenth of real time - but needs no special build).
`--fair-sched=yes` is essential there or the firmware busy-wait starves
the sim thread; the wrapper sets it.

## Running

```
obj/AM32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
```

Options:

- `--config FILE` JSON file with motor/battery/esc/sim properties (see
  `example.json`, all keys optional)
- `--eeprom FILE` eeprom backing file (default `am32_eeprom.bin`). A
  missing file is seeded with the AM32 configurator default settings
- `--can-uri URI` CAN interface, default `mcast:0` (group 239.65.82.N
  port 57732, wire compatible with libcanard and ArduPilot SITL). An
  optional interface may be given (`mcast:0:lo`). `none` disables CAN,
  for pure PWM/DShot testing
- `--node-id N` force the DroneCAN node ID, otherwise DNA is used
- `--input-port N` UDP port for PWM/DShot input (default 57733, 0
  disables)
- `--state-port N` UDP port for high rate simulation state streaming
  and runtime motor model loading (default 57734, 0 disables)
- `--bind-any` bind the input and state ports on all interfaces instead
  of loopback only, needed when the GUI runs on a different host. The
  ports accept unauthenticated control, so only use on trusted networks
- `--input-type N` force the eeprom INPUT_SIGNAL_TYPE setting (0=auto
  1=dshot 2=servo 5=dronecan)
- `--speedup X` simulation speed relative to wall clock, 0 = free running
- `--uid STR` string used to derive the 16 byte unique ID
- `--verbose` 1Hz state line on stderr
- `--nosleep` busy wait instead of sleeping. Uses two full CPU cores but
  avoids OS sleep/wakeup latency for the most accurate wall clock pacing
- `--bootloader ELF` chain with the bootloader SITL from the
  am32-bootloader repo: the process boots into the bootloader first
  (as hardware does) and every reset lands back in it with the right
  reset cause; the bootloader execs this firmware on jump. The two
  share the eeprom file, the input/CAN ports, and `<eeprom>.bkup` (the
  RTC backup registers carrying the DroneCAN firmware-update handoff).
  The bootloader keeps its flash in `<eeprom>.blflash`. Default off

The virtual ESC can then be controlled with the DroneCAN GUI tool or
pydronecan on `mcast:0`. A test script is included:

```
python3 Mcu/SITL/sitl_can_test.py --throttle 0.5 --duration 20
```

which arms, ramps the throttle via `esc.RawCommand` and reports the
`esc.Status` telemetry (RPM, voltage, current, temperature).

Note that with no DroneCAN traffic directed at the node it will reboot
every 2 seconds from the firmware signal-timeout logic, exactly as real
hardware does. Reboots (including `RestartNode` and watchdog resets)
re-exec the process; the eeprom file persists.

Each instance holds a lock on its eeprom file: two instances sharing an
eeprom (and therefore a node ID) would interleave their DroneCAN
transfers on the bus, which shows up as erratic telemetry. To run
multiple ESCs give each its own `--eeprom` and `--node-id`.

## PWM/DShot input over UDP

The SITL listens on a UDP port (default 57733) for PWM or DShot input
frames. Each packet is one frame on the virtual signal wire, synthesized
into input-capture edge timestamps and decoded by the firmware's
unmodified `Src/signal.c`/`Src/dshot.c` logic, including input type
auto-detection, CRC checking, zero-throttle arming, DShot commands and
bidirectional DShot auto-detect (idle high line).

packet format (little endian):

| field | size | meaning |
|-------|------|---------|
| magic | u16  | 0x4453 |
| type  | u8   | 0=PWM 1=DSHOT150 2=DSHOT300 3=DSHOT600 4=SERIAL19200 5=LINE_LEVEL |
| len   | u8   | payload bytes after the header (4; for type 4 the number of serial bytes, 1..200) |
| flags | u16  | bit0: line idle level (1 = idle high, bidir DShot; for type 5 the constant level), bit1: line floating (types 4/5) |
| data  | u16  | PWM pulse width in us, or the full 16 bit DShot frame; for type 4 replaced by the raw serial bytes |

Bidirectional DShot replies (eRPM plus extended telemetry frames) are
sent back to the most recent sender in the same format, with `data`
carrying the 16 bit GCR-decoded reply frame.

Type 4 carries raw bytes framed as 19200 baud 8N1 on the simulated
signal wire and type 5 sets a constant line state (driven high, driven
low, or floating). Both exist for the bootloader SITL (see the
am32-bootloader repo), which bit-bangs the 4-way configuration protocol
on the signal pin and detects the input type from the line state at
boot; the main firmware ignores type 4 and uses type 5 only as the idle
line level. The bootloader replies with type 4 packets carrying its
serial output. `sitl_fourway.py` implements the one-wire client side.

### Configurators against the SITL

There are two ways a configurator reaches an ESC, and the SITL emulates
both. Through a flight controller it talks MSP, asks for BLHeli 4-way
passthrough, and the FC translates each 4-way command into the ESC's
one-wire bootloader protocol; `msp_stub_fc.py` is that flight
controller. Direct - what the am32-configurator calls direct mode and
the Offline-Configurator drives at 19200 baud - it talks the bootloader
protocol itself, through a 1-wire USB linker soldered onto the signal
wire; `sitl_serial_bridge.py` is that linker. Either way an unmodified
configurator reads and writes the settings and flash of a simulated ESC.

The GUI does all of this for you: give the **SITL process** panel a
bootloader as well as a binary, start it, and pick **USB 4-way** or
**USB serial** from the mode box. The status next to it becomes the
serial port to hand to the configurator, and **No USB device** gives it
back. Picking a mode for a simulator this GUI started without a
bootloader is refused rather than left to fail silently - both modes
end up at the ESC bootloader, and the application answers neither
protocol. The rest of this section is what that does, for running it by
hand.

**USB serial (direct)** uses a separate linker USB identity so am32.ca
automatically chooses its direct protocol at 19200 baud. After changing
USB modes, use the browser's **Port select** to select the newly attached
device before connecting. Direct mode reaches only the **Direct ESC** target.

Run the SITL chained with the bootloader, then the stub:

```
obj/AM32_AM32_SITL_CAN_*.elf --can-uri none \
    --bootloader <am32-bootloader>/obj/AM32_SITL_BOOTLOADER_PB4_CAN_*.elf
python3 Mcu/SITL/msp_stub_fc.py --no-motor --verbose
```

The stub prints the pty to use as the serial port, e.g.

```
SerialPortConnector_CLI settings /dev/pts/7
```

`--esc-ports` lists the input ports of several SITL instances to serve
as separate ESCs on the 4-way interface, `--no-motor` leaves the DShot
output off (it would otherwise share the signal wire with the 4-way
session), and an ESC that is running the application instead of the
bootloader is reset into it over the state port, the way a real FC
power cycles one.

#### Direct mode: the 1-wire linker

`sitl_serial_bridge.py` takes the FC out of the picture and pipes the
serial port straight to the signal wire:

```
python3 Mcu/SITL/sitl_serial_bridge.py --verbose
```

It prints a pty the same way, takes the same `--usbip` options, and
resets a running ESC into the bootloader on the first command just as
the 4-way path does. Two details make it behave like the hardware
rather than like a socket:

- a real linker shorts TX to RX, so the host reads back everything it
  sent before the ESC answers. Both configurators use that echo to find
  the start of a response and the web one requires it, so the bridge
  reproduces it. `--no-echo` turns it off for a client that cannot cope.
- the echo comes back at 19200 baud, not instantly. The bootloader
  separates a command from the buffer upload that follows it by the
  line idle in between, and a configurator only sends that buffer once
  it has read the echo of the command - so echoing early would let the
  two run together into one frame the bootloader cannot parse. Bytes
  the host hands over while the wire is still busy continue the current
  frame; bytes that arrive after it has drained start a new one.

#### A virtual USB serial device

A pty is enough for tools that open a port by path, but not for a
browser: Chrome's Web Serial only lists what the kernel enumerated as a
USB serial device. `--usbip` therefore serves the same byte stream as a
simulated USB CDC-ACM adapter over the USB/IP protocol
(`sitl_usbip.py`), which the Linux `vhci_hcd` driver attaches as a real
device:

```
sudo modprobe vhci_hcd
python3 Mcu/SITL/msp_stub_fc.py --usbip --attach --no-motor
```

The device then appears in `dmesg`, as `/dev/ttyACM*` and as
`/dev/serial/by-id/usb-AM32_AM32_SITL_serial_SITL-if00`, and is
indistinguishable from hardware to anything above the driver - Chrome
included. It enumerates as pid.codes `1209:0001`, which the AM32
configurator accepts as a flight controller. `sitl_serial_bridge.py`
takes the same options and serves the same device.

vhci_hcd is handed the socket to speak USB/IP over rather than opening
it itself, and does not care what kind it is, so the export defaults to
an abstract unix socket (`@am32-sitl-usbip.<uid>`): no port for anything
to collide with, nothing reachable from the network, and nothing left in
the filesystem if the process is killed. `--usbip-socket` names a
different one, a name without a leading `@` being a filesystem path,
which is the one to use when the socket should be protected by its
permissions rather than open to the network namespace. `--usbip-port`
exports over tcp instead, for a client on another machine or one that
can only attach the `usbip` way.

`--attach` does the import and the attach itself (re-running itself
under sudo for the sysfs write, since only that needs root), so the
usbip userspace package is not required; without it the command to run
by hand is printed. `python3 Mcu/SITL/sitl_usbip.py --detach` detaches
everything again, as does `sudo usbip detach -p 0`.

A second instance needs its own `--usbip-serial`, since udev names the
`/dev/serial/by-id` link after the usb serial string.

Windows has no equivalent in the box: attaching a remote USB/IP device
needs a signed client driver, so a bridge or a virtual COM port driver
is the practical route there for now.

Tools in `Mcu/SITL/`:

- `sitl_gui.py` — Qt (PySide6) GUI driving both the PWM/DShot input and
  DroneCAN input with per-input enable switches (for failover testing),
  BDShot/EDT and esc.Status telemetry with rates, and an
  `INPUT_SIGNAL_TYPE` parameter panel. The simulation panel selects the
  motor model (the JSON files in `Mcu/SITL/models/`, applied to the
  running simulation over the state port; switch at zero throttle for
  clean results). **Create...** next to the model list opens a builder
  that turns a motor spec sheet — size code (`2216` = 22mm x 16mm
  stator), poles, Kv, and optionally winding resistance, no-load idle
  current, weight, propeller and power source — into a new model.json,
  so a new setup can be simulated without knowing torque constants and
  inductances (`model_builder.py` is the same logic on the command
  line). The **SITL process** panel runs the simulator binary itself,
  on the ports this GUI drives, instead of starting it separately: pick
  the binary (one is bundled with the packaged build, or Browse), an
  eeprom and an optional bootloader, and Start; leave it stopped to
  drive a simulator you ran yourself. The USB mode box in the same panel
  presents the running ESC to configurators as a virtual USB serial
  device (see above), either as **USB 4-way** through the fake flight
  controller or as **USB serial** straight onto the signal wire; it
  stops the DShot input while either is on, since a configurator
  session and a DShot stream cannot share one signal wire. The
  simulation panel also has optional high rate views, both default off:
  pyqtgraph scopes of the phase currents and the phase terminal
  voltages, each in its own window (sample period down to the 500ns
  physics step and adjustable window; the sample rate is automatically
  limited to about 200k samples/s of wall clock, so fine periods take
  effect as the speedup is lowered — the PWM dead time diode conduction
  is visible on the voltages at fine sample periods), and a motor/bridge
  animation showing rotor angle, per phase bridge modes and the
  comparator. A speedup slider (0.01x to 2x)
  changes the simulation pace at runtime, for watching the animation in
  slow motion; input frames arriving faster than the slowed simulation
  consumes them are dropped, as on a real wire. A stuck rotor slider
  blocks the prop with a virtual obstruction (think of a branch caught
  in the prop), from free through partial drag to completely stuck, for
  exercising the firmware's `STUCK_ROTOR_PROTECTION`; the Release
  button clears it. `--control-port N` accepts UI
  commands over a localhost TCP connection for scripted tests (default
  off); `--log FILE` records every UI action with timestamps and
  `--replay FILE` plays a recording back, so a failing interactive
  session can be reproduced exactly. Install the
  dependencies (PySide6, pyqtgraph, dronecan; Linux/Windows/macOS) into
  a self-contained environment with

```
python3 Mcu/SITL/make_gui_env.py
```

  which creates `Mcu/SITL/venv` and prints the interpreter to run the
  GUI with. A system python with the packages from
  `Mcu/SITL/requirements.txt` installed works too.
  `python3 Mcu/SITL/build_sitl_gui.py` packages the GUI into a single
  executable with the SITL bundled (so it runs the simulator out of the
  box); CI builds one for Linux and Windows. The UI backends
  live in `sitl_gui_backend.py`, UI-independent for headless tests
- `sitl_serial_bridge.py` — 1-wire USB linker emulation: a serial port
  (a pty, or a virtual USB serial device) piped straight to the signal
  wire, which is the configurators' direct mode.
- `msp_stub_fc.py` — fake Betaflight FC: MSP on a pty (or on a virtual
  USB serial device, `sitl_usbip.py`), DShot to the SITL, and BLHeli
  4-way passthrough to the simulated ESC bootloader
  (`sitl_fourway_server.py`). Used by `scripts/esc_capture_fc.py` for
  hardware-free telemetry capture and by configurators for settings and
  flashing
- `dshot_test.py` — headless scripted test (arming, throttle, EDT,
  bad-CRC injection), e.g.:

```
obj/AM32_AM32_SITL_CAN_*.elf --can-uri none --input-type 1
python3 Mcu/SITL/dshot_test.py --type dshot600 --bidir --edt --throttle 800
```

Note that the eeprom default `INPUT_SIGNAL_TYPE` is DRONECAN_IN, which
disables the PWM/DShot input interrupts at startup — set it to 0/1/2
first (via `--input-type`, the GUI parameter panel, or
`dshot_test.py --input-type`). Also be aware of the current firmware
input arbitration: once any `esc.RawCommand` has been received, the 1kHz
DroneCAN input keep-alive overrides the `dshot`/`inputSet` flags and
PWM/DShot input is dead until a reboot (signal timeout after the CAN
stream stops) followed by zero-throttle re-arming. Running both inputs
at once exercises exactly this behaviour, which is what the input
priority/failover parameter work is developing against.

## macOS

Builds and runs natively (Apple Silicon or Intel) with the stock Xcode
command line tools: `make AM32_SITL_CAN`. The GUI bootstrap is the same
`python3 Mcu/SITL/make_gui_env.py`. Multicast CAN over loopback works
without configuration.

## Windows

The SITL builds under Cygwin (packages: gcc-core, make) with the same
`make AM32_SITL_CAN`, producing a native console executable, and the
POSIX signal based scheduler runs correctly under the Cygwin runtime.
The GUI uses a normal Windows python: `py Mcu/SITL/make_gui_env.py`
creates the environment and prints the interpreter to use; after that
`sitl_gui.bat` in the repository root launches the GUI (double click or
from cmd, extra arguments are passed through). Notes:

- to run the binary from outside a Cygwin shell (cmd, double click),
  copy `C:\cygwin64\bin\cygwin1.dll` next to it - its only Cygwin
  dependency (the CI artifact ships it bundled).
- Windows Firewall must allow inbound UDP for the SITL binary (or ports
  57732-57734) for CAN and the input/state ports to receive.
- a socket never receives its own multicast on Windows, so the CAN TX
  self test is skipped there; on a machine with several interfaces pass
  an explicit one as `--can-uri mcast:0:<ip>`.

## Headless / CI use

Everything runs without a display: the SITL is a plain console binary
and the GUI works under Qt's offscreen platform
(`QT_QPA_PLATFORM=offscreen`) driven through `--control-port`, so full
interactive scenarios can run in CI. On a minimal Debian/Ubuntu the
requirements are:

```
apt install gcc make python3 python3-venv \
    libgl1 libegl1 libfontconfig1 libxkbcommon0
pip install dronecan        # for the DroneCAN tests
python3 Mcu/SITL/make_gui_env.py   # for GUI-driven tests
```

Multicast CAN over loopback works on a stock VM with no route
configuration (the SITL self-tests its TX at startup). Timing notes for
slow or virtualised runners: the simulation paces itself and reports
the achieved ratio in `--verbose` (x1.00 = real time); the python test
senders keep their average frame rate under coarse sleep granularity by
sending catch-up bursts, which matters because the firmware's
bidirectional DShot auto-detect needs more than 100 frames before
zero-throttle arming completes, putting a floor of roughly 100Hz on the
usable frame rate.

## Debugging with gdb

The SITL debugs like any host program, with one wrinkle: emulated
interrupts are delivered by parking the firmware thread with SIGUSR1,
which gdb must pass through silently. `Mcu/SITL/gdbinit` sets that up:

```
gdb -x Mcu/SITL/gdbinit --args obj/AM32_AM32_SITL_CAN_*.elf --can-uri none --input-type 1
(gdb) break tenKhzRoutine
(gdb) run
```

Attach to a running simulator with `gdb -x Mcu/SITL/gdbinit -p <pid>`.
Stopping (a breakpoint, single stepping, ^C) freezes the firmware and
the simulation together: simulated time, the emulated watchdog and the
ESC's own timeouts all stop with the process, and on resume the pacer
rebases to the wall clock instead of sprinting through the backlog, so
pausing under the debugger never breaks the physics. For deterministic
stepping of firmware code with simulated time frozen between steps use
`set scheduler-locking step`.

## Architecture

The firmware runs unmodified (built as `am32_main()`) in one thread. A
simulation thread owns simulated time, advancing it in fixed physics
steps (500ns default) and delivering emulated interrupts (BEMF
comparator, commutation timer, 20kHz loop timer, CAN RX) by suspending
the firmware thread with a signal and running the handler, reproducing
the run-to-completion interrupt semantics of the real MCU. Simulated
time is decoupled from wall time and paced to `--speedup`.

The emulated MCU follows the G431 target: TIM1 PWM generation with
preloaded ARR/CCR, a 2MHz interval timer, one-shot commutation timer,
20kHz loop timer and 1MHz utility timer, plus comparator/EXTI blanking
behaviour matching `Mcu/g431`. Timer reads from interrupt context step
the physics, so `delayMicros()` inside handlers and the comparator
filter loop behave as on hardware.

The motor model (`sim/motor.c`) is a trapezoidal back-EMF BLDC model
based on open-bldc-csim: per-phase currents, solved star point voltage
(so the floating phase terminal voltage and its zero crossings are
physical), fet Rds_on, body diode clamping of a floating phase carrying
current, and battery voltage sag from internal resistance. The
comparator compares the floating phase against the virtual neutral with
configurable noise and hysteresis, so the firmware's blanking and
filtering logic is genuinely exercised at PWM switching level.

### Building the Windows ZIP locally

From this checkout on Linux, run:

```
python3 Mcu/SITL/build_win11.py
```

This copies the current working sources (including uncommitted edits) to
`win11:am32-sitl-gui-build`, builds there through its Cygwin SSH shell, and
retrieves `dist/am32-sitl-gui-windows.zip`. It does not commit or push.
The remote machine needs Cygwin `gcc-core`, `make`, `python3`, `git`, `rsync`
and native Windows Python 3.12 (`py -3.12`). The Python build environment is
created automatically from `windows/requirements-build.txt`. The packaged
GUI is tested before retrieving the ZIP; add `--test-usb` to also test a real
virtual COM port when the bundled USBip driver is installed on the build host.
Use `--host`, `--remote-dir`, `--python` or
`--ssh-config` to override the defaults.

The bootloader defaults to the upstream commit pinned in `build_win11.py`.
Use `--bootloader-source ../AM32-bootloader` to test a different checkout,
including its local edits. Select another Windows host firmware or bootloader
with the GUI's Browse controls; ARM hardware ELF files cannot execute on the
host. The packaged defaults include both host executables and their Cygwin
runtime, so end users do not need development tools.

The pinned bootloader needs `windows/bootloader-cygwin.patch` for Windows
multicast and to calculate the seeded flash checksum after setting its firmware
name. The build applies it in a disposable copy and links the bootloader
below 4 GB without address randomization, because its device-info protocol
uses 32-bit addresses. The original bootloader checkout is not modified.

CI runs the same script with `--local` under Cygwin, then uploads the common
`dist/windows-package` contents as `am32-sitl-gui-windows`. The local ZIP and
the CI download have the same layout and packaging inputs; their executable
bytes and ZIP timestamps can differ with source, compiler and dependency
versions. `package_windows.py` downloads the checksum-pinned USB/IP installer
from the ArduPilot mirror and includes it with a CRLF `README.txt`. Install
that prerequisite and reboot to enable the GUI's Windows COM port support.

### Betaflight App

Use the **ESCs** selector above the tabs to choose 1–8 independent ESCs;
the default is one. Each **ESC N** tab has its own motor controls, plots,
model and persistent EEPROM. Choosing or editing the SITL binary or bootloader
in any tab selects it for all ESCs, including tabs added later. These changes
apply when the simulators are next started. New tabs inherit ESC 1's input
options. **Start all** and **Stop all** operate the whole
bench; each tab also retains its individual Start/Stop buttons. DShot is
the default launch input. Stop all simulations and select **No USB device**
before changing the ESC count.

The shared **USB 4-way** and **USB Betaflight** connections advertise the
configured count to am32.ca and Betaflight. Motor/ESC numbers match the tab
numbers. Direct USB wiring reaches only the ESC chosen in **Direct ESC**;
select that target while USB is off. Disconnect the browser before changing
USB modes. After editing ESC settings, use **Stop all**, then **Start all**
to reload them.

Extra EEPROM files use `.esc2.bin` through `.esc8.bin` beside the first
default EEPROM and survive tab removal and recreation. Each ESC must use a
different EEPROM file. Signal/state ports increase by 10 per ESC (default
ESC 2: 57743/57744). CAN controls use separate multicast buses, starting at
`--can-uri mcast:N` and increasing by one per tab, to avoid cross-control or
node-ID collisions. Multiple tabs support multicast CAN or `--can-uri none`.
Use `--esc-count 8` to start the GUI with eight tabs. The control socket
accepts `esc 8 ds_value 500` to address a tab; unprefixed commands address
ESC 1, while `esc_count`, `sim_start_all` and `sim_stop_all` address the bench.

Betaflight USB mode uses the emulated STM32 VCP ID `0483:5740`, which
Betaflight's default serial-port chooser accepts. The USB product remains
`AM32 SITL serial`. After updating the Python sources, restart the GUI and
select this mode again to re-enumerate the device with the new ID.

Select **USB Betaflight (motor control)** in the GUI, start the simulator
with **Input: DShot**, and connect [Betaflight App](https://app.betaflight.com)
to the displayed serial port. The Setup tab reports a stationary, level
accelerometer and gyro. In Motors, enable motor testing and use each motor's or
the master slider. Leave throttle at zero for two seconds after startup.
RPM, temperature, voltage and current are decoded from the simulated ESC's
actual bidirectional DShot/EDT replies.

The simulated FC supports MSP API 1.46, MSPv1, native MSPv2 and MSPv2-over-v1.
Motors can select DSHOT150/300/600, bidirectional DShot and motor pole count,
then Save and Reboot. Enable Auto-Connect in the app, or reconnect after
reboot. Changing the DShot rate or polarity also restarts the simulated ESC
to detect the new signal. The CLI supports `get`, `set`, `save`, `exit`, `version`
and `status`, with `motor_pwm_protocol`, `dshot_bidir`, `dshot_edt`
(OFF/ON/FORCE) and `motor_poles`. Betaflight also sends an EDT-enable command
when motor testing starts, regardless of the saved `dshot_edt` setting.
DShot direction commands reach AM32 through the simulated signal wire.

GUI DShot and CAN controls are disabled while Betaflight owns motor control.
The output stops after two seconds without MSP requests; CLI entry, reboot
and 4-way passthrough also stop motor testing. Disconnect the app before
switching USB modes. The existing USB 4-way mode remains available for ESC
configuration and flashing without a motor command stream.

FC settings persist in `<selected EEPROM>.fc.json`, separately from the
AM32 EEPROM. Standalone `msp_stub_fc.py --config FILE` provides the same
persistence. `--esc-ports` drives and reports up to eight ESCs; each state
port defaults to the corresponding signal port plus the first ESC's
state/signal port offset. Motor ordering is fixed in ESC tab order.
PID/filter fields are retained for Motors
tab compatibility; there is no flight dynamics or PID loop emulation.
GPS/barometer/magnetometer, full CLI/configurator coverage, and Betaflight FC
flashing are unsupported. A bootloader is needed for ESC passthrough, but
not for Betaflight motor testing.

Run the protocol regressions with:

```sh
python3 -m unittest discover -s Mcu/SITL -p test_msp_betaflight.py -v
python3 Mcu/SITL/multi_esc_gui_test.py
python3 Mcu/SITL/multi_esc_sitl_test.py
```

Pass `--bootloader PATH` to the multi-ESC integration test to also exercise
4-way discovery and independent settings writes. On Linux, `--usb` tests
the same traffic over a real USB/IP serial device (requires passwordless
sudo for attach/detach).
