# sigrok-pico
Warning: This is all very much in development!!!
Use a raspberry pi pico (rp2040) as a logic analyzer and oscilloscope with sigrok
This is not just a "dump a trace to text file" but uses the pico SDK CDC serial library to communicate with sigrok-cli/pulseview through a driver.

sigrok-generic is the sigrok driver hardware library.  Originally I hoped to create a "standard" protocol description as to how various MCUs might communicate with sigrok, but it's unlikely I'll ever enable anything but a pico.

pico_pgen is a simple digital function generator useful for creating patterns to test.

pico_sdk_sr_gen1 is the pico sdk C code that uses DMA, the PIO and ADC to sample up to 3 analog channels and 21 logic channels.
