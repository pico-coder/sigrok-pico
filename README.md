# NOTE:
# At this time my pull request into the main sigrok repo has not been accepted.
# Thus mainline releases of sigrok cli and pulseview do not support this repo.

The pull request is https://github.com/sigrokproject/libsigrok/pull/181 . I have no ETA when it may be pulled.

I have managed to do a cross compile of PulseView for windows.  Instructions to download are here.
https://github.com/pico-coder/sigrok-pico/blob/main/pulseview/Readme.md

For linux, at least three other people have managed to combine my pull request into a libsigrok build. If you are on linux it's probably a practical experiment to try.  4GB of RAM is recommended for pulseview builds to avoid disk swap issues.  See SigrokBuildNotes.md

#
# sigrok-pico
Use a raspberry pi pico (rp2040) as a logic analyzer and oscilloscope with sigrok.
This implementation uses the pico SDK CDC serial library to communicate with sigrok-cli/pulseview through a sigrok driver.

## Directories:

pico_pgen is a simple digital function generator useful for creating patterns to test.

pico_sdk_sigrok is the pico sdk C code for the PICO RP2040 device.

The latest libsigrok code exists as a fork at https://github.com/pico-coder/libsigrok

## Files
PICOBuildNotes.md - build notes for building the PICO device assuming you have gone through the PICO C SDK "getting started with PICO".

SigrokBuildNotes.md - rough libsigrok build notes which will be depracated once raspberrypi_pico is mainline

GettingStarted.md - quick run down on setting things up.

AnalyzerDetails.md - details on supported modes of the analyzer and various limitations.

SerialProtocol.md - details of the "over the wire" protocol used between the sigrok driver and the device.
