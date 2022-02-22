# sigrok-pico
Use a raspberry pi pico (rp2040) as a logic analyzer and oscilloscope with sigrok
This implementation uses the pico SDK CDC serial library to communicate with sigrok-cli/pulseview through a sigrok driver.

raspberrypi_pico is the libsigrok directory that should be under libsigrok/src/hardware.
Create the new device with 
>sigrok-util/source $ ./new-driver "RaspberryPI PICO"


pico_pgen is a simple digital function generator useful for creating patterns to test.

pico_sdk_sr_gen1 is the pico sdk C code that uses DMA, the PIO and ADC to sample up to 3 analog channels and 21 logic channels.
