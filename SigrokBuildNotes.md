Building this should follow build flows similar to the documented build flows in libsigrok.
Note: These steps rely on a brute-force copy approach to move the 3 needed files under /libsigrok/src/hardware/raspberrypi-pico.  A cleaner way would be to pull a specific repo as the first step that has the sigrok-pico code in it, but this method helps ensure you can do a normal baseline build.  Once this gets officially released into sigrok mainline then the sigrokutil/ new driver steps can be skipped.

Do something like this:
1) git clone the mainline libsigrok, install all dependent libraries and build a baseline libsigrok.
2) git clone the mainline sigrok-utils as described in sigrok build flows. 
     Use the mainline sigrok-util. There is no special sigrok-pico version of it because there are no files that are modified.  Instead sigrok-util just creates patch for libsigrok.
4) cd <path>/sigrok-util/source
5) ./new-driver "RaspberryPI PICO"
6) copy api.c, protocol.c and protocol.h from https://github.com/sigrokproject/libsigrok/pull/181 into <path>/libsigrok/src/hardware/raspberrypi-pico
7) cd <path>/libsigrok
8) ./autogen.sh
9) ./configure
10) make

Install libsigrok and then build the sigrokcli and pulseview per the documentation.
