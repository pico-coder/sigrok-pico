Running sigrok cli or pulseview at debug level 2 is highly recommended to see reported issues with user configurations.

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
2 triggering modes are supported, both of which only trigger on the digital inputs.
### HW triggered via PIO
This mode is only supported if all of the analog inputs are disabled. This is because there is the HW triggering is
based on stalling the PIO until a value is seen on pin D2.  There is no good interlock to stall ADC DMA at the same time
so the device will ignore this mode if it is requested along with analog channels. 

D2 is the only supported trigger pin, and  is only level (not edge/change) sensitive.

The trigger mode is specified via a special encoding of the sample rate because I don't know of any other libsigrok configuration to pass arbitrary values from the user to the device. (If you know of one, please let me know).
The special config bits are specified in the lowest decimal character of the sample rate.  The value is specified as (sample_rate%10)&0x6, i.e. the remainder of dividing sample rate by 10 and then bitwise 
AND with 0x6 to create a 3 bit value.

- Bit 0 must be left at 0 (since Pulseview requires even sample rates)
- Bit 1 sets the level value
- Bit 2 enables the HW triggering

Thus a sample rate of 10000 does not enable HW triggering, but a value of 10006 enables HW triggering with a level of 1 and a sample rate of 10000.
      
Note that the first trigger value seen will not be captured because it is used to unstall the PIO.

### SW triggered via libsigrok 
Any one or more enabled digital pins can be use for triggering in this mode.  Only digital pins are used for triggering, but analog is captured in sync with the digital triggers.

SW triggers supports level, rising, falling, and changing across all channels. The triggers are specified in the triggering definition of pulseview or sigrokcli. 
A pre-capture ratio of 0 to 100% can also be specified. The pre capture buffer will only fill to the ratio if sufficient pretrigger samples are seen.
Note that SW triggering adds substantial host side processing, and such processing on slower processors may limit the maximum usable streaming sample rate.

### HW and SW modes can be combined
This can only be used for digital only capture because HW triggering doesn't support analog.
The HW trigger must occur first for the device to start sending data, and then the SW trigger is applied to the samples received at the host.

### Always trigger
If neither a software or hardware trigger are specified, then the device will immediately capture data.

## Storage Modes
The device supports two modes of storage of samples, Fixed Depth and Continuous streaming.  Fixed depth is enabled if the trigger mode is Always Trigger or HW trigger and the requested number of samples fits in the device storage space.
Continuous streaming is enabled at all other times, thus any time SW triggering is enabled, or the number of samples is greater than the internal depth.  
Fixed depth is a preferred mode because it guarantees that the device can store the samples and send them to host as USB transfer rates allow.  
In continuous streaming it is possible that the required bandwidth to issue the samples is greater than the available USB bandwidth.  
Thus the user must make a tradeoff between guaranteed capture of limited depth, or larger depths with possible loss.  
The device can detect overflow cases in Continuous Streaming mode and send an abort code to the host which is reported to Pulseview.  
Abort cases in Continous stream will cause the total number of samples to be reduced, but should not allow corrupted values to be sent.

## Sample rate
Hint: Updates to the sample_rate in pulseview don't take effect until you hit enter, so do that before clicking run or you might capture with the old sample rate.

To provide flexibility, the user is given the ability to specify sample rates that may be beyond the capacity of the device to store internally or to transfer to the host in time. 
It is up to the user to understand the limitations below and use accordingly.

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

The protocol supports a 4 channel digital mode which supports run length encoding on the wire.  
Assuming a high frequency sample rate of a relatively on low duty factor signals, the 4 channel mode may allow Continous Streaming and SW triggering of signals that may not otherwise be possible.

## Sample rate hard limits
### Common sample rate
The PIO and ADC share a common sample rate.  This is because libsigrok only supports a common rate and because it keeps the device DMA implementation sane.
### Sample rate granularity 
The sample rate granularity is limited to the a granularity of the clock divisors of the PIO and ADC.  The ADC uses only the integer part of the fractional divisor of the 48Mhz USB clock.
The PIO uses the full fractional divisor of the sysclk which is set to 120Mhz (the maximum supported PIO rate). 
Pulseview allows for any arbitrary sample rate but the device code calculates the nearest divisor, so the actual sample rates are unkown unless the UART0 debug port is captured.
It is recommended that for digital modes that integer divisors of 120Mhz be specified to get sample rates close to what is specified (i.e. 120,60,40,30,20,15,12,10Msps etc).
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
In the normal mode, each 7 bits of digital data takes one byte, and each analog sample takes a byte.  So a 12 bit digital trace with 2 analog channels takes 4 bytes per sample.
At 400KB/sec, Continous Streaming would be limited to around 100Ksps.


## Debug UART
The hardware UART0 prints debug information to UART0 TX at 115200bps.  
Since the sigrok driver on the host tries to report most user errors it is not required for use.  However, if you are filing a bug sighting having that output could be very useful.
