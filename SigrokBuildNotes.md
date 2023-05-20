Building this should follow build flows similar to the documented build flows in libsigrok.
Note: These steps rely on a brute-force copy approach to move the 3 needed files under /libsigrok/src/hardware/raspberrypi-pico.  A cleaner way would be to pull a specific repo as the first step that has the sigrok-pico code in it, but this method helps ensure you can do a normal baseline build.  Once this gets officially released into sigrok mainline then the sigrokutil/ new driver steps can be skipped.

Do something like this:
1. git clone the mainline libsigrok, install all dependent libraries and build a baseline libsigrok.
1. git clone the mainline sigrok-utils as described in sigrok build flows. 
     Use the mainline sigrok-util. There is no special sigrok-pico version of it because there are no files that are modified.  Instead sigrok-util just creates patch for libsigrok.
1. `cd <path>/sigrok-util/source`
1. `./new-driver "RaspberryPI PICO"`
1. `cd <path>/libsigrok`
1. `git am ../sigrok-util/source/0001-raspberrypi-pico-Initial-driver-skeleton.patch`
1. copy api.c, protocol.c and protocol.h from https://github.com/sigrokproject/libsigrok/pull/181 into <path>/libsigrok/src/hardware/raspberrypi-pico
     ```
     cd src/hardware/raspberrypi-pico/
     wget https://raw.githubusercontent.com/sigrokproject/libsigrok/7aa830f96d0d088e147ae30d2122c35539207dcc/src/hardware/raspberrypi-pico/protocol.h
     wget https://raw.githubusercontent.com/sigrokproject/libsigrok/7aa830f96d0d088e147ae30d2122c35539207dcc/src/hardware/raspberrypi-pico/protocol.c
     wget https://raw.githubusercontent.com/sigrokproject/libsigrok/7aa830f96d0d088e147ae30d2122c35539207dcc/src/hardware/raspberrypi-pico/api.c
     ```
1. `./autogen.sh`
1. `./configure`
1. `make`

Install libsigrok and then build the sigrokcli and pulseview per the documentation.
