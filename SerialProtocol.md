# Protocol Overview
The serial transfers can be grouped into 4 flows.
1) A configuration and control protocol for setting up sample rates, channel enables, triggering etc.
2) A general data transfer protocol for any configuration that uses analog channels or more than 4 digital channels
3) An optimized protocol for digital only configures with 4 or fewer channels that includes run length encoding.
4) A final byte count used at the end of the general data transfer or optimized protocol 
# Configuration and Control - Host to Device
The '*' and '+' are the only two characters that are commands by themselves and do not have any acknowledgement from the device.  All other control signals require a character followed by \n or \r, and expect a response as described in each command.

'*' - An asterisk sent from the host to device is a reset of the current sampling state of the device.  It terminates any pending sampling and data transfers and clears sample counts and puts the device in an idle state.  It does not effect the USB CDC/Serial link state. The sigrok device driver sends this as the first part of the initial device scan, and at the start of all acquisitions.

'+' - A plus sent from the host to the device indicates a host forced abort of an in progress capture, which can only be initiated by clicking the stop button in pulseview during an in progress capture.  It terminates the sending of data, but does not restore as much state as a reset.
# Configuration and Control commands the require a response with data.  
These commands require the device to return with a character string.  If the device considers the command to be incorrect, no response is sent and the host driver will timeout and error.

'i' - Identify.  This is sent from the sigrok scan function to identify the device.  The device replys with a string of the format "SRPICO,AxxyDzz,00". The "SRPICO," is a fixed identifier.  The "Axx" value is the letter 'A' followed by a two character decimal field identifying the number of analog channels.  The y indicates the number of bytes that are used to send analog samples across the wire, for now only the value of 1 is supported. The "Dzz" is the letter 'D' followed by a two character decimal field indicating the number of digital channels supported.  The final "00" indicates a version number , which for now is always "00". Thus the full featured 3 analog and 21 digitial channel build returns "SRPICO,A03D21,00".

'a' - Analog Scale and offset.  The host sends a "Ax" where is X is the channel number, asking the device what scale and offset to apply the sent value to create a floating point value.  The device returns with a string of the format "aaaaxbbbbb", where the "aaa" represent the scale in uVolts, the x is the letter 'x', and "bbbb" represent the offset in uVolts. Both the scale and offset can be variable length up to a combined 18 characters. Both scale and offset should support negative signs.
# Configuration and Control commands that respond with ack.  
If the device receives these commands and considers the values appropriate it returns a single "*", otherwise is returns nothing and the device driver will timeout in error.

'R' - Sets the sample rate. The 'R' is followed by a decimal value string indicating the sample rate, such as "R100000".

'L' -Sets the sample limit, i.e. the total number of samples. The 'L' is followed by a decimal value string indicating the number of samples, such as "L5000".

'A' - Analog channel enable.  These are of the format "Axyy" where x is 0 for disabled, 1 for enabled and yy is the channel number.  Thus "A103" enables analog channel 3.

'D' - Digital channel enable.  These are of the format "Dxyy" where x is 0 for disabled, 1 for enabled and yy is the channel number.  Thus "D020" disables analog channel 20.
# Configuration and Control commands with no response.  
These commands do not expect an acknowledgement of any kind because they initiate data capture/transfer.

'F' - Fixed Sample mode - tells the device to grab a fixed set of samples.  This is used in all cases where SW based triggering is not enabled.

'C' - Continous Sample mode - tells the device to continuously transfer data because SW triggering is processing the data stream to find a trigger.

# Device to host commands.
'!' - Device detected abort - this is the only command sent by the device that is not initiated by a command from the host.  It is used in cases where the device has detected a capture overflow condition and is no longer sending more data.  The device will periodically send this until the host sends a '*' or '+'.

# General Data transfer protocol.
This is used for all cases where any analog channels are enabled, or more than 4 digital channels are enabled.
Samples of digital and analog data are sent in groups where for a given point in time the values of each channel are sent.  A sample for each digital and analog channel is sent as one slice of information, rather than sending all of the sample data for a given channel at once.  Data samples are sent first where each transmitted byte is the value of a group of 7 digital channels,OR'd with 0x80 so that no ASCII control characters are used.  Lowest channels are sent first.  Each enabled analog channel is then sent where it's 7 bit sample value is also OR'd with 0x80 to avoid ASCII control characters.

For example, assume 14 digital channels (D2 to D15) and 2 analog channels, a "slice" of sample data might be sent as: 0x8F, 0xA3, 0x91, 0xB6.

That would indicate Digital Channels 8:2 are 0xF, Channels 15:9 are 0x23, Analog channel A0 is 0x11 and Analog Channel A1 is 0x36.

# Optimized 4 Digital channel protocol with Run Length Encoding (RLE).
There are many narrow width high speed protocols (I2C,I2S,SPI) which may require sample rates higher than the 300kB to 500kB transfer rates supported by the Serial CDC interface.  For cases where transactions are in bursts of activity surrounded by low activity, a run length encoding scheme is enabled to reduce wire transfer bandwidth and enable sampling rates higher than that supported by the protocol.


