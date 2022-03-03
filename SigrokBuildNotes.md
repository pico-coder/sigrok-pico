Building this should follow build flows similar to the documented build flows in libsigrok.

So something like this:
1) git clone libsigrok, install all dependent libraries and build a baseline libsigrok.
2) git clone the sigrok-utils as described in sigrok build flows.
3) cd <path>/sigrok-util/source && ./new-driver "RaspberryPI PICO"
4) copy api.c, protocol.c and protocol.h from https://github.com/sigrokproject/libsigrok/pull/181 into <path>/libsigrok/src/hardware/raspberrypi-pico
5) cd <path>/libsigrok
6) ./autogen.sh
7) ./configure
8) make

Install libsigrok and then build the sigrokcli and pulseview per the documentation.
