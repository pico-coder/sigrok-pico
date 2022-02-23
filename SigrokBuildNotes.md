Building this should follow build flows similar to the documented build flows in libsigrok.
So something like this:
1) git clone libsigrok, install all dependent libraries and build a baseline libsigrok.
2) git clone the sigrok-utils as described in sigrok build flows.
3) cd <path>/sigrok-util/source $ ./new-driver "RaspberryPI PICO"
4) copy the 3 files under raspberrypi-pico to <path>/libsigrok/src/hardware/raspberrypi-pico
5) cd <path>/libsigrok
6) ./autogen.sh
7) ./configure.sh
8) make

Install libsigrok and then build the sigrokcli and pulseview per the documentation.
