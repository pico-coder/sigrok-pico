#ifndef SR_DEVICE_H
#define SR_DEVICE_H
#include <stdint.h>
#include <stdbool.h>

// Pin usage
// GP0 and 1 are reserved for debug uart
// GP2-GP22 are digital inputs
// GP23 controls power supply modes and is not a board input
// GP24-25 are not on the board and not used
// GP26-28 are ADC.
// number of analog channels
#define NUM_A_CHAN 3
// number of digital channels
#define NUM_D_CHAN 21
// Mask of bits 22:2 to use as inputs -
#define GPIO_D_MASK 0x7FFFFC
// Storage size of the DMA buffer.  The buffer is split into two halves so that when the first
// buffer fills we can send the trace data serially while the other buffer is DMA'dinto
#define DMA_BUF_SIZE 220000
// The size of the buffer sent to the CDC serial
// The TUD CDC buffer is only 256B so it doesn't help to have more than this.
#define TX_BUF_SIZE 260
#define UART_BAUD 921600
// This sets the point which we will send data from the txbuf to the usb cdc.
// For the 5-21 channel RLE it must leave a spare ~83 entries to cover the case where
// a new long steady input comes after deciding to not send a sample.
//(Assuming 128KB samples per half, a max rle value of 1568 we can get
//   256*1024/2/1568=83 max length rles on a steady input).
// Other than that the value is not very specific because the usb tub code
// implement a 256 entry fifo that queues things up and sends max length 64B transactions
// 20 is arbitrarly picked to ensure that if we have even a little we send it so that
// at least something goes across the link.
#define TX_BUF_THRESH 20
// Base value of sys_clk in khz.  Must be <=125Mhz per RP2040 spec and a multiple of 24Mhz
// to support integer divisors of the PIO clock and ADC clock
#define SYS_CLK_BASE 120000
// Boosted sys_clk in khz.  Runs the part above its specifed frequency limit to support faster
// processing of digital run length encoding which in some cases may allow for faster
// streaming of digital only data.
//****************************
// Use this at your own risk
//***************************
// Frequency must be a 24Mhz multiple and less than 300Mhz to avoid known issues
// The authors PICO failed at 288Mhz, but testing with 240Mhz seemed reliable
// #define SYS_CLK_BOOST_EN 1
// #define SYS_CLK_BOOST_FREQ 240000

typedef struct
{
   uint32_t sample_rate;
   uint32_t num_samples;
   uint32_t a_mask, d_mask;
   uint32_t samples_per_half; // number of samples for one of the 4 dma target arrays
   uint8_t a_chan_cnt;        // count of enabled analog channels
   uint8_t d_chan_cnt;        // count of enabled digital channels
   uint8_t d_tx_bps;          // Digital Transmit bytes per slice
   // Pins sampled by the PIO - 4,8,16 or 32
   uint8_t pin_count;
   uint8_t d_nps; // digital nibbles per slice from a PIO/DMA perspective.
   uint32_t scnt; // number of samples sent
   char cmdstrptr;
   char cmdstr[20];                                             // used for parsing input
   uint32_t d_size, a_size;                                     // size of each of the two data buffers for each of a& d
   uint32_t dbuf0_start, dbuf1_start, abuf0_start, abuf1_start; // starting memory pointers of adc buffers
   char rspstr[20];
   // mark key control variables voltatile since multiple cores might access them
   volatile bool started;
   volatile bool sending;
   volatile bool cont;
   volatile bool aborted;
   /*Depracated trigger logic
 //If HW trigger enabled, uncomment all usages
   //volatile bool notfirst;  //Have we processed at least a first sample (so that lval is correct
   //  volatile bool triggered;
   //  uint32_t tlval; //last digital sample value - must keep it across multiple calls to send_slices for trigger
   //  uint32_t lvl0mask,lvl1mask,risemask,fallmask,chgmask;
   End depracated trigger logic*/
} sr_device_t;

// initialize debug uart
int Dprintf(const char *fmt, ...);

// Process incoming character stream
int process_char(sr_device_t *d, char charin);

// reset as part of init, or on a completed send
void reset(sr_device_t *d);

// initial post reset state
void init(sr_device_t *d);

// Initialize the tx buffer
void tx_init(sr_device_t *d);

// Process incoming character stream
// Return 1 if the device rspstr has a response to send to host
// Be sure that rspstr does not have \n  or \r.
int process_char(sr_device_t *d, char charin);

#endif /* SR_DEVICE_H */
