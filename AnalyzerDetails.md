Running sigrok cli or pulseview at debug level 2 is highly recommended to see reported issues with user configurations.

![alt text](https://github.com/pico-coder/sigrok-pico/blob/main/PICO_Sigrok_Rates.PNG)
## Channels

### 21 Digital Channels 
Digital channels are PICO board pins D2-D22 and are named accordingly in sigrok. Channels must be enabled (via Pulseview or sigrokcli) starting at D2 and continuously set towards D22

### 3 Analog Channels
Any combinations of A0 (ADC0 pin 31),A1(ADC1 pin 32) and A2(ADC2 pin 34) can be enabled.
Channels are only 7 bit accurate because:
    1) Even though the ADC gives a 12 bit value, the ENOB of the RP2040 is only ~8 bits.
    2) 7 bits makes an easy wire encoding that avoids ASCII characters that can be messed up by serial drivers (see SerialProtocol.md)
    3) 7 bits still gives 20mV accuracy and 128 divisions which is usually plenty of separation.

### Disabling channels
Note that disabling any unused channels will often reduce serial transfer overhead and allocate more trace storage for the enabled signals, so always disabled unused channels.
  

## Trigger Modes
HW triggering via PIO was supported in earlier versions, but removed in rev2 because:
1) There was some issue where it seemed to false trigger when the triggering condition wasn't present.
2) There was no clear way to specify HW vs Sw triggering in pulseview, especially when the change was made to specify sample rate via a dropdown selection.
If needed you could clone the code for the device and reenable the one line that enabled the triggering.  
Note that in the rev2 release the triggers are now sent to the device.  The device doesn't use them because the processing overhead for HW triggering is so high that it reduces the effective streaming rate below that of continuous software triggering on the host, and thus adds no value.

### SW triggered via libsigrok 
Any one or more enabled digital pins can be use for triggering in this mode.  Only digital pins are used for triggering, but analog is captured in sync with the digital triggers.

SW triggers supports level, rising, falling, and changing across all channels. The triggers are specified in the triggering definition of pulseview or sigrokcli. 
A pre-capture ratio of 0 to 100% can also be specified. The pre capture buffer will only fill to the ratio if sufficient pretrigger samples are seen.
Note that SW triggering adds substantial host side processing, and such processing on slower processors may limit the maximum usable streaming sample rate.

### Always trigger
If no trigger is specified, then the device will immediately capture data of a fixed length.

## Storage Modes
The device supports two modes of storage of samples, Fixed Depth and Continuous streaming.  Fixed depth is enabled if the trigger mode is Always Trigger and the requested number of samples fits in the device storage space.
Continuous streaming is enabled at all other times, thus any time SW triggering is enabled, or the number of samples is greater than the internal depth.  
Fixed depth is a preferred mode because it guarantees that the device can store the samples and send them to host as USB transfer rates allow.  
In continuous streaming it is possible that the required bandwidth to issue the samples is greater than the available USB bandwidth.  
Thus the user must make a tradeoff between guaranteed capture of limited depth, or larger depths with possible loss.  
The device can detect overflow cases in Continuous Streaming mode and send an abort code to the host which is reported to Pulseview.  
Abort cases in Continous stream will cause the total number of samples to be reduced, but should not allow corrupted values to be sent.

## Sample rate
For better usability, the user is given a fixed set of sample rates in pulseview.  The user is given the ability to specify sample rates that may be beyond the capacity of the device to store internally or to transfer to the host in time. 

It is up to the user to understand the limitations below and use accordingly, but the host driver will limit sample rates and print errors and warnings when limits are exceeded.

If multiple ADC channels are enabled, the specified sample rate is the rate per channel. 
Since the RP2040 only supports one ADC conversion at a time the actual ADC rate must 2x or 3x the specified sample rate for 2 channel or 3 channel ADC.  
Note that while pulseview displays the ADC samples together in time, they are actually staggered due to the single conversion limitation.

Sample rate limitations come in two types, hard and soft.
Hard limitations are intrinsic to the how the device functions and are enforced by the libsigrok driver.
If the user specifies sample rates the break the hard limits the driver will report error error messages from Pulseview and/or the sigrok cli
if level 2 verbosity or higher is set.

Soft limitations are recommendations that if not followed may cause aborts.
Specifically they are related to the ability of the device to send sample data across the USB serial interface before the DMA updates overflow the storage buffers.
Soft limitations are detected by the device when it sees a sample overflow issue and sends an abort signal to the host.  

The protocol supports a run length encoding (RLE) for all digital only sample modes which reduces the amount of data sent on the wire.  
Assuming a high frequency sample rate of a relatively on low duty factor signals, RLE may allow Continous Streaming and SW triggering of signals that may not otherwise be possible.

## Sample rate hard limits
### Common sample rate
The PIO and ADC share a common sample rate.  This is because libsigrok only supports a common rate and because it keeps the device DMA implementation sane.
### Sample rate granularity 
The sample rate granularity is limited to the a granularity of the clock divisors of the PIO and ADC.  The ADC uses only the integer part of the fractional divisor of the 48Mhz USB clock.
The PIO uses the full fractional divisor of the sysclk which is set to 120Mhz (the maximum supported PIO rate). 
Pulseview only provides frequencies which yield integer divisors for the PIO and ADC clocks to ensure that the two clocks do not drift over time which can happen with non integer divisors.  
The command line interface will allow any specified frequency. It is recommended that for digital modes that integer divisors of 120Mhz be specified to get sample rates close to what is specified (i.e. 120,60,40,30,20,15,12,10Msps etc).
As the sample rate is decreased the granularity increases.
Since the ADC has a maximum frequency of 500khz off a 48Mhz clock it has a minimum divisor of 96 and thus a worst case granularity of 5khz.
### Min and Max Sample rates.
The minimum sample rate is 5Khz to ensure the sample rate is within the 16 bit divisor.  
The maximum sample rate for digital only is 120Msps.  
If 8 or more digital channels are enabled sample rates of 60Msps or less are recommended to allow the DMA engine to do a read modify write operation from the PIO FIFO to memory.  
Faster rates might work, or they might not...
## Sample rate soft limits
Soft limits are hard to quantify because they can be based on the maximum USB link bandwidth, or the ability of the device to process and send samples, or on the ability of the host to process samples (especially in SW trigger modes).
Based on testing with a Raspberry PI Model 3B+, the USB port reaches a maximum of 300KB-400KB/sec on the 12Mbit USB link.  
Soft limits can be completely avoided by not using SW triggers and setting trace depths that enable Fixed Sample modes, but a particular protocol may not be practical to trace with those restrictions.
In the D4 optimized RLE mode, each byte on the wire holds a 4 bit sample value and a 0-7 sample RLE value, or an 8-640 sample RLE value.  
Thus in this mode, it should be possible to Continuous Stream at a sample rate of 300ksps regardless of the sampled data activity.  If the activity factor is less, then higher sample rates are possible.
For instance a 20% AF signal has been captured at a sample rate of 2 Msps.
In the other digital only modes, each groups of 7 channels or sent in one byte and a one byte RLE encoding is used.
In mixed digital/analog or analog only modes, each 7 bits of digital data takes one byte, and each analog sample takes a byte.  So a 12 bit digital trace with 2 analog channels takes 4 bytes per sample.

## Debug UART
The hardware UART0 prints debug information to UART0 TX at 115200bps in rev1, and 921600 for rev2 and beyond.
Since the sigrok driver on the host tries to report most user errors it is not required for use.  However, if you are filing a bug sighting having that output could be very useful.
