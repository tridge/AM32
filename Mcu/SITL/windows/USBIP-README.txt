USB/IP prerequisite for AM32 SITL and ESCSim
==========================================

Package: USBip-0.9.7.7-x64.exe (33,226,344 bytes)
Project: USB/IP Client for Windows, by Vadim Grinshpun and contributors
Source: https://github.com/vadimgrn/usbip-win2

Many thanks to Vadim Grinshpun and all usbip-win2 contributors for developing
and maintaining this open-source USB/IP client and Windows drivers. Their
work makes USB support in AM32 SITL and ESCSim possible.

Release: https://github.com/vadimgrn/usbip-win2/releases/tag/v.0.9.7.7
Original download:
https://github.com/vadimgrn/usbip-win2/releases/download/v.0.9.7.7/USBip-0.9.7.7-x64.exe
SHA-256:
51620fa5f9f8be5932bc9d786deee557ce06d5407a99cab490dcfac71f185fea

This is the unmodified upstream x64 installer, also pinned by ESCSim's
scripts/build-windows-installer.py. The driver is attestation signed.
Version 0.9.7.7 fixes enumeration of the full-speed USB device used by SITL.
Use this tested version. ESCSim rejects 0.9.7.8 due to driver crashes.
The BSD-2-Clause license is in usbip-win2-LICENSE.txt.

Mirror:
https://firmware.ardupilot.org/Tools/AM32-tools/ESCSim/USBIP/

Run the installer, approve Windows' administrator prompt, and reboot before
using USB in the simulator. No test-signing mode is needed with this package.
