It is recommended that you read through the AnalyzerDetails.md for specifics on modes of the device, but if you just can't wait:

1) Get some 1Kohm or greater resistors to put inline between the PICO inputs, you don't want to accidentally fry your PICO because you put in voltages <0V or >3.3V or accidentally jumpered ground to VCC. 
2) Get the [sigrok-pico/pico_sdk_sigrok/build/pico_sdk_sigrok.uf2](./pico_sdk_sigrok/build/pico_sdk_sigrok.uf2) file and program it to your PICO devices (examples are online, everywhere).
3) Install Pulseview and skip to step 7, or sigrok-cli - follow descriptions from the sigrok pages for both.
4) Replug the PICO to reset it, and then use sigrok-cli to scan for avaliable serial ports
```
sigrok-cli --list-serial
```
5) Use sigrok-cli to scan for the device based on the serial port you found above. The baudrate doesn't matter because we are CDC serial.
```
sigrok-cli  -l 2 -d raspberrypi-pico:conn=/dev/ttyACM0:serialcomm=115200/flow=0 --scan
```
6) Do a first trace 
```
~/github/sigrok-cli/sigrok-cli  -l 2 -d raspberrypi-pico:conn=/dev/ttyACM0:serialcomm=115200/flow=0 --config samplerate=10000  --channels D2,D3,D4,D5 --samples 1000
```
7) Aternatively to using sigrok-cli use Pulseview.
8) Go read AnalyzerDetails.md like you should have to begin with....
