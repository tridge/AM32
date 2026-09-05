AM32 SITL for Windows (64-bit)
=============================

1. Extract the entire ZIP with Explorer's "Extract All".
2. Double-click am32-sitl-gui.exe.
3. In "SITL process", choose DShot in the Input box and click "Start
   simulator". The firmware, bootloader, EEPROM and motor models are already
   included. Python, a compiler and a Cygwin installation are not required.
4. Enable the DShot pane, hold zero throttle for at least two seconds to
   arm, then adjust throttle. To use the DroneCAN pane instead, choose
   DroneCAN in the Input box before starting the simulator.

USB configurator support (one-time installation)
------------------------------------------------
USB support is optional for running the motor simulation.

1. Close work using USB devices; the driver installer restarts USB hubs.
2. Run USBIP\USBip-0.9.7.7-x64.exe and approve the administrator prompt.
   Install the USB/IP client and driver using the default options.
3. Reboot Windows after installation, even if the installer does not ask.
   The supplied driver is signed; do not enable Windows test-signing mode.
4. Launch am32-sitl-gui.exe normally and start the simulator.
5. Select "USB 4-way (fake FC)" in the SITL process panel. Its status will
   show a COM port when ready. The GUI attaches the device automatically.
6. Open https://am32.ca or https://am32.tridgell.net in Chrome or Edge.
   Connect to the displayed COM port and read the ESC settings.
7. Disconnect the configurator before choosing "No USB device", switching
   USB modes or closing the GUI. "USB serial (direct)" is for configurators
   that support a direct one-wire USB linker.
8. After editing ESC settings, select "No USB device", then Stop and Start
   the simulator to load the settings (equivalent to power-cycling an ESC).

No physical ESC, flight controller or USB cable is needed. GUI DShot transmission
is stopped when USB is enabled because both use the same simulated wire.
If no COM port appears, check the USB status for an error, reboot after driver
installation, and try again. Use the supplied USBip version. If installed to
a custom folder, set USBIP_EXE to the full path to usbip.exe before launching.

Choosing other firmware and bootloaders
--------------------------------------
Stop the simulator, use Browse next to "SITL binary" or "Bootloader", then
start it again. These must be Windows host SITL executables built from AM32
and AM32-bootloader. Build outputs may have an .elf extension despite being
Windows executables. Hardware ARM ELF/HEX images and Linux executables cannot
be run here. A bootloader must be selected for ESC configuration/flashing access.

Settings are saved in %LOCALAPPDATA%\AM32-SITL\eeprom.bin. The bootloader's
flash and backup files are stored alongside it. They survive GUI upgrades.
Use EEPROM Browse to choose a different writable image or to run separate
instances. To restore defaults, stop the GUI and rename the AM32-SITL folder.

This simulates firmware compiled for the host CPU. Uploading a hardware HEX
through a configurator exercises flash storage; it does not replace the host
program running the simulation. Select another host SITL binary with Browse
to run different firmware code.

USBIP\README.txt records the USB/IP installer origin and checksum.

Betaflight App motor control
---------------------------
1. Start the simulator with Input set to DShot.
2. Select "USB Betaflight (motor control)" and note the COM port.
3. Open https://app.betaflight.com in Chrome or Edge, select that port and
   connect. The Setup tab shows a stationary simulated accelerometer/gyro.
4. Open Motors. Motor 1 is the simulated ESC. Enable motor testing and
   raise its slider (or the master slider). Allow two seconds at zero
   throttle after startup for the ESC to arm.
5. DSHOT150/300/600, Bidirectional DShot and motor pole count can be changed
   in Motors, then saved with Save and Reboot. Enable Auto-Connect in the
   app, or reconnect after reboot. RPM and EDT telemetry come
   from AM32. Other motor outputs and flight dynamics are not simulated.
6. In CLI, use "set dshot_edt = ON" (or OFF/FORCE), "set dshot_bidir = ON",
   "set motor_pwm_protocol = DSHOT300", or "set motor_poles = 14", then
   "save". Only this small CLI subset is supported. Betaflight also sends
   EDT-enable when you turn on motor testing, independently of dshot_edt.
7. Disconnect Betaflight before switching USB modes. GUI DShot and CAN
   controls are disabled in this mode; the app owns motor control. Motor
   output returns to zero if MSP polling stops for two seconds.

FC settings are saved alongside the selected EEPROM as <eeprom>.fc.json;
these are separate from AM32 settings in the ESC EEPROM. For the bundled
EEPROM both files live under %LOCALAPPDATA%\AM32-SITL. PID/filter fields
are compatibility placeholders, and do not simulate a flight controller's
control loop. GPS, barometer and magnetometer are absent. Flash AM32 using
its configurator and the USB 4-way mode; Betaflight FC flashing is unsupported.
