/*Raspberry Pi PICO/RP2040 code to implement a logic analyzer and oscilloscope
 * Some Original code from the pico examples project:
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/stdlib.h" //uart definitions
#include <stdlib.h> //atoi,atol, malloc
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
//#include "hardware/sio.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "tusb.h"//.tud_cdc_write...

#include "sr_device.h"

//forced_test_mode is a special mode that puts the device into an active sampling
//state out of reset.  It is used for a quick way to debug features without needed
//pulseview or sigrok-cli to initiate a transfer.
//The enable means we will start the mode once, the run indicates it is currently running.
//To enable the feature, set forced_test_mode_en to true, but leave forced_test_mode_run false.
bool forced_test_mode_en=false; //true;
bool forced_test_mode_run=false; 
PIO pio = pio0;
uint piosm=0;
uint8_t *capture_buf;
sr_device_t dev;
volatile uint32_t tstart;
volatile bool send_resp=false;

uint8_t txbuf[TX_BUF_SIZE];
uint16_t txbufidx;
uint32_t rxbufdidx,rxbufaidx;
uint32_t rlecnt;
uint32_t bytecnt=0; //count of characters sent serially
#ifdef PIN_TEST_MODE
  struct repeating_timer pt_timer;
  uint32_t pin_test_cnt=0;
  //The systick code is used to determine whether the 100us is reliably
  //being generated.  If it is not then the test samples can look "stretched"
  #define SYSTICK_SIZE 128 
  #define SYSTICK_PRINT 64
  uint32_t systick_array[SYSTICK_SIZE];
  uint32_t systick_idx=0;
#endif //PIN_TEST_MODE
//Number of bytes stored as DMA per slice, must be 1,2 or 4 to support aligned access
//This will be be zero for 1-4 digital channels.
uint8_t d_dma_bps; 
uint32_t samp_remain;
uint32_t lval,cval; //last and current digital sample values
uint32_t num_halves; //track the number of halves we have processed
uint32_t exp_halves; //the number of halves we expect in non-continous mode
uint32_t halves_seen=0;
uint admachan0,admachan1,pdmachan0,pdmachan1;
//While 1 maintenance DMA could write the pio and adc DMAs, they may complete
//at slightly different times, and based on channel configurations an adc or pio 
//may not be enabled so dedicate a maintence channel for each.
uint amaintchan0,amaintchan1,pmaintchan0,pmaintchan1;
dma_channel_config acfg0,acfg1,pcfg0,pcfg1,amcfg0,amcfg1,pmcfg0,pmcfg1;
//Two addresses for adc and pio dma engines
//These are read by the maintenance DMA engines and written to the 
//PIO/ADC DMA engine write addrs
uint32_t *amaddrs[2]={0,0};
uint32_t *pmaddrs[2]={0,0};
uint32_t tmpaddr0,tmpaddr1; //temp variables for address generation
uint32_t *tmpptr;
uint32_t dma_halves; //number of halves observed by the dma int handler
volatile bool mask_xfer_err;
int usbintin;
uint8_t uartch;//rx uart character -ignored as only uart tx is used
uint8_t h0intmask,h1intmask; //The required masks of ints for each half
//Keeps tracks of all interrupts we received.  Sometimes we might get an analog and not a digital
//DMA int (or vice versa) and need to exit that handler but need to clear out pending interrupt state
//so this stores what we have seen.
uint8_t currintmask; //The current mask of received ints.
uint32_t sho_cnt; //number of times we entered the loop
uint32_t tx_cnt; //number of times we did any kind of send
uint32_t acnt,bcnt,ccnt,dcnt,ecnt;

void print_DMA(){
  //Print out the read addr, write addr, transaction count, and control/status 
  //of the lowest 8 DMA controllers. Note that relative to RP2040, the RP2350 moved the 
  //location of busy and chain_to (and maybe others)
  /*
  tmpptr=(uint32_t *)DMA_BASE;
  for(int i=0;i<8;i++){
    //note that in pointer math +1 adds 4B for a uint, so this is offset 0,4,8,c
    //and the 0x10 multiple is really 0x40
     Dprintf("RA%d 0x%X\n\r",i,*(tmpptr+0x10*i));
     Dprintf("WA%d 0x%X\n\r",i,*(tmpptr+0x10*i+1));
     Dprintf("TA%d %d\n\r",i,*(tmpptr+0x10*i+2));
     Dprintf("CA%d 0x%X\n\r",i,*(tmpptr+0x10*i+3));
*/
  }

void print_DMA_chan(int chan){
  //Print out the read addr, write addr, transaction count, and control/status 
  //of one channel
  tmpptr=(uint32_t *)DMA_BASE;
  //note that in pointer math +1 adds 4B for a uint, so this is offset 0,4,8,c
  //and the 0x10 multiple is really 0x40
   Dprintf("RA%d 0x%X\n\r",chan,*(tmpptr+0x10*chan));
   Dprintf("WA%d 0x%X\n\r",chan,*(tmpptr+0x10*chan+1));
   Dprintf("TA%d %d\n\r",chan,*(tmpptr+0x10*chan+2));
   Dprintf("CA%d 0x%X\n\r",chan,*(tmpptr+0x10*chan+3));
}
//The function stdio_usb_out_chars is part of the PICO sdk usb library.
//However the function is not externally visible from the library and rather than
//figure out the build dependencies to do that it is just copied here.
//This is much faster than a printf of the string, and even faster than the puts_raw
//supported by the PICO SDK stdio library, which doesn't allow the known length of the buffer
//to be specified. (The C standard write function doesn't seem to work at all).
//This function also avoids the inserting of CR/LF in certain modes.
//The tud_cdc_write_available function returns 256, and thus we have a 256B buffer to feed into
//but the CDC serial issues in groups of 64B.  
//Since there is another memory fifo inside the TUD code this might possibly be optimized
//to directly write to it, rather than writing txbuf.  That might allow faster rle processing
//but is a bit too complicated.

void my_stdio_usb_out_chars(const char *buf, int length) {
    static uint64_t last_avail_time;
    uint32_t owner;
// See https://github.com/pico-coder/sigrok-pico/pull/63/.  
//tud_ready does not rely on DTR
// so use it rather than tud_cdc_connected
//    if (tud_cdc_connected()) {
    if (tud_ready()) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int) tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                int n2 = (int) tud_cdc_write(buf + i, (uint32_t)n);
                tud_task();
		            tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
            		tud_cdc_write_flush();
//                if (!tud_cdc_connected() || -replaced per pull request 63
                if (!tud_ready() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
}

//A common init for all send_slice modes 
void send_slice_init(sr_device_t *d,uint8_t *dbuf){
   rxbufdidx=0;
   rxbufaidx=0;
   txbufidx=0;
   rlecnt=0;
   //Adjust the number of samples to send if there are more in the dma buffer
   samp_remain=d->samples_per_half;
   if((d->cont==false)&&((d->scnt+samp_remain)>(d->num_samples))){
        samp_remain=d->num_samples-d->scnt;
        d->scnt+=samp_remain;
        //Dprintf("SSIa sph %d scnt %d lval 0x%X rem %d ns %d\n\r",d->samples_per_half,d->scnt,lval,samp_remain,d->num_samples);
   }else{
        d->scnt+=d->samples_per_half;
        //Dprintf("SSIb sph %d scnt %d lval 0x%X rem %d ns %d\n\r",d->samples_per_half,d->scnt,lval,samp_remain,d->num_samples);
   }
}

//This is an optimized transmit of trace data for configurations with 4 or fewer digital channels 
//and no analog.  Run length encoding (RLE) is used to send counts of repeated values to effeciently utilize 
//USB CDC link bandwidth.  This is the only mode where a given serial byte can have both sample information
//and RLE counts.
//Samples from PIO are dma'd in 32 bit words, each containing 8 samples of 4 bits (1 nibble).
//RLE Encoding:
//Values 0x80-0xFF encode an rle cnt of a previous value with a new value:
//  Bit 7 is 1 to distinguish from the rle only values.
//  Bits 6:4 indicate a run length up to 7 cycles of the previous value
//  Bits 3:0 are the new value.
//For longer runs, an RLE only encoding uses decimal values 48 to 127 (0x30 to 0x7F)
//as x8 run length values of 8..640.
//All other ascii values (except from the abort and the end of run byte_cnt) are reserved.
uint32_t send_slices_D4(sr_device_t *d,uint8_t *dbuf){
   uint8_t nibcurr,niblast;
   uint32_t cword,lword; //current and last word
   uint32_t *cptr;
   //Note that this function always sends the first 8 samples, even if
   //send_slice_init sets remaining samples to zero.  That shouldn't happen
   //as we should also be in free running mode or split the two halves
   //into something with 8 samples.
   send_slice_init(d,dbuf);
   //Don't optimize the first word (eight samples) perfectly, just send them to make the for loop easier, 
   //and setup the initial conditions for rle tracking
   cptr=(uint32_t *) &(dbuf[0]);
   cword=*cptr;
   #ifdef D4_DBG
   Dprintf("Dbuf %p cptr %p data 0x%X\n\r",(void *)&(dbuf[0]),(void *) cptr,cword);
   #endif
   lword=cword;
   for (int j=0;j<8;j++){
     nibcurr=cword&0xF;
     txbuf[j]=(nibcurr)|0x80;
     cword>>=4;
   }
   niblast=nibcurr;      
   cptr=(uint32_t *) &(txbuf[0]);
   txbufidx+=8;
   rxbufdidx+=4;
   rlecnt=0;
   //Note that it is generally assumed that each half buffer has far more than
   //8 samples in it, especially if pulseview is running.  But for some useages it
   //may be only 8 so exit on the first 8. This is just mostly to prevent underflow
   //of samp_remain when we subtract 8 from it.
   if(d->samples_per_half<=8){
     my_stdio_usb_out_chars(txbuf,txbufidx);
     d->scnt+=d->samples_per_half;
     return txbufidx;
   }
   //The total number of 4 bit samples remaining to process from this half.
   //Subtract 8 because we procesed the word above.
   samp_remain-=8;

   //Process one  word (8 samples) at a time.
   for(int i=0;i<(samp_remain>>3);i++) {
       cptr=(uint32_t *) &(dbuf[rxbufdidx]);
       cword=*cptr;
       rxbufdidx+=4;
       #ifdef D4_DBG2
       Dprintf("dbuf0 %p dbufr %p cptr %p \n\r",dbuf,&(dbuf[rxbufdidx]),cptr);
       #endif
       //Send maximal RLE counts in this outer section to the txbuf, and if we accumulate a few of them
       //push to the device so that we don't accumulate large numbers
       //of unsent RLEs.  That allows the host to process them gradually rather than in a flood
       //when we get a value change.
       while(rlecnt>=640){
         txbuf[txbufidx++]=127;
         rlecnt-=640;
         if(txbufidx>3){   
            my_stdio_usb_out_chars(txbuf,txbufidx);
            bytecnt+=txbufidx;
            txbufidx=0;
         }
       }
       //Coarse rle looks across the full word and allows a faster compare in cases with low activity factors
       //We must make sure cword==lword and that all nibbles of cword are the same
       if((cword==lword)&&((cword>>4)==(cword&0x0FFFFFFF))){
         rlecnt+=8;
         #ifdef D4_DBG2
         Dprintf("coarse word 0x%X\n\r",cword);
         #endif
       }
       else{//if coarse rle didn't match
         #ifdef D4_DBG2
        Dprintf("cword 0x%X nibcurr 0x%X i %d rx idx %u  rlecnt %u \n\r",cword,nibcurr,i,rxbufdidx,rlecnt);
        #endif
        lword=cword;
        for (int j=0;j<8;j++){ //process all 8 nibbles
          nibcurr=cword&0xF;
          if(nibcurr==niblast) {
             rlecnt++;
          }
          else{
            //If the value changes we must push all remaing rles to the txbuf
            //chngcnt++;
            //Send intermediate 8..632 RLEs
            if(rlecnt>7) {
	       int rlemid=rlecnt&0x3F8;
               txbuf[txbufidx++]=(rlemid>>3)+47;
            } 
            //And finally the 0..7 rle along with the new value
            rlecnt&=0x7;
            #ifdef D4_DBG2 //print when sample value changes
 	       Dprintf("VChang val 0x%X rlecnt %d i%d j%d \n\r",nibcurr,rlecnt,i,j);
            #endif		  
            txbuf[txbufidx++]=0x80|nibcurr|rlecnt<<4;
            rlecnt=0;
	  }//nibcurr!=last
          cword>>=4;
          niblast=nibcurr;
        }//for j
       } //else (not a coarse rle )
       #ifdef D4_DBG2
       Dprintf("i %d rx idx %u  rlecnt %u \n\r",i,rxbufdidx,rlecnt);
       Dprintf("i %u tx idx %d bufs 0x%X 0x%X 0x%X\n\r",i,txbufidx,txbuf[txbufidx-3],txbuf[txbufidx-2],txbuf[txbufidx-1]);
       #endif
       //Emperically found that transmitting groups of around 32B gives optimum bandwidth
       if(txbufidx>=64){
         my_stdio_usb_out_chars(txbuf,txbufidx);
         bytecnt+=txbufidx;
         txbufidx=0;
       }
    }//for i in samp_send>>3
    //At the end of processing the half send any residual samples as we don't maintain state between the halves
    //Maximal 640 values first
    while(rlecnt>=640){
      txbuf[txbufidx++]=127;
      rlecnt-=640;
    }
    //Middle rles 8..632
    if(rlecnt>7) {
      int rleend=rlecnt&0x3F8;
      txbuf[txbufidx++]=(rleend>>3)+47;
    }
    //1..7 RLE 
    //The rle and value encoding counts as both a sample count of rle and a new sample
    //thus we must decrement rlecnt by 1 and resend the current value which will match the previous values
    //(if the current value didn't match, the rlecnt would be 0).
    if(rlecnt){
      rlecnt&=0x7;
      rlecnt--;
      txbuf[txbufidx++]=0x80|nibcurr|rlecnt<<4;
      rlecnt=0;
    }
    if(txbufidx){
       my_stdio_usb_out_chars(txbuf,txbufidx);
       bytecnt+=txbufidx;
       txbufidx=0;
    }

}//send_slices_D4

//Send a digital sample of multiple bytes with the 7 bit encoding
void inline tx_d_samp(sr_device_t *d,uint32_t cval){
    for(char b=0;b < d->d_tx_bps;b++){
      txbuf[txbufidx++]=(cval|0x80);
//      Dprintf("txds b %d cv 0x%X idx %d \n\r",b,cval,txbufidx); 
      cval>>=7;
    }
}

//Allow for 1,2 or 4B reads of sample data to reduce memory read overhead when
//parsing digital sample data.  This function is correct for all uses, but if included
//the compiled code is substantially slower to the point that digital only transfers
//can't keep up with USB rate.  Thus it is only used by the send_slices_analog which is already
//limited to 500khz, and in the starting send_slice_1/2/4
uint32_t  get_cval(uint8_t *dbuf){
       uint32_t cval;
       if(d_dma_bps==1){
           cval=dbuf[rxbufdidx];
       }else if(d_dma_bps==2){
           cval=(*((uint16_t *) (dbuf+rxbufdidx)));
       }else{
           cval=(*((uint32_t *) (dbuf+rxbufdidx)));
           //Mask off undefined channels
           #ifdef DIG_26_MODE
             //push out the gap of 3 channels
              cval=(cval&MEM_D_MASK_L)|((cval&MEM_D_MASK_U)>>3);
           #elif BASE_MODE
              //mask off upper unused
               cval=cval&MEM_D_MASK_L;
              //No change for DIG_32_MODE as all are defined  
           #endif
       }
       rxbufdidx+=d_dma_bps;
       return cval;
}
/*RLE encoding for 5 or more channels has two ranges.
Decimal 48 to  79 are RLEs of 1 to 32 respectively.
Decimal 80 to 127 are (N-78)*32 thus 64,96..80,120..1568
Note that it is the responsibility of the caller to
forward txbuf bytes to USB to prevent txbufidx from overflowing the size
of txbuf. We do not always push to USB to reduce its impact
on performance.
 */
void inline check_rle(){
//  Dprintf("RLEx %d\n\r",rlecnt); 
  while(rlecnt>=1568){
    txbuf[txbufidx++]=127;
    rlecnt-=1568;
  }
  if(rlecnt>32){
     uint16_t rlediv=rlecnt>>5;
     txbuf[txbufidx++]=rlediv+78;//was 86;
     rlecnt-=rlediv<<5;
    }
  if(rlecnt){
     txbuf[txbufidx++]=47+rlecnt;
     rlecnt=0;
  }
}

//Send txbuf to usb based on an input threshold
void check_tx_buf(uint16_t cnt){
  if(txbufidx>=cnt){
//     Dprintf("txbuf idx %d cnt %d\n\r",txbufidx,cnt);
     my_stdio_usb_out_chars(txbuf,txbufidx);
     bytecnt+=txbufidx;
     txbufidx=0;
  }
}
//A common first digital byte to send to establish RLE.
//Not used for send_analog because it doesn't use RLE, and not used for D4 because it 
//has a different RLE encoding
void send_first_dig_sample(sr_device_t *d,uint8_t *dbuf){
   lval=get_cval(dbuf);
   tx_d_samp(d,lval);
   samp_remain--;
   rlecnt=0;
}
//There are three very similar functions send_slices_1B/2B/4B.
//Each of which  is very similar but exist because if a common function 
//is used with the get_cval in the inner loop, the performance drops 
//substantially.  Thus each function has a 1,2, or 4B aligned read respectively.
//We can't just always read a 4B value because the core doesn't support non-aligned accesses.
//These must be marked noinline to ensure they remain separate functions for good performance
//1B is 5-8 channels
void __attribute__ ((noinline)) send_slices_1B(sr_device_t *d,uint8_t *dbuf){
   send_slice_init(d,dbuf);
//   Dprintf("Enter 1Ba sts %d sr %d\n\r",d->samples_per_half,samp_remain); 
   send_first_dig_sample(d,dbuf);
   for(int s=0;s<samp_remain;s++){
       cval=dbuf[rxbufdidx++]; 
       if(cval==lval){
           rlecnt++;
         }
       else{
//         Dprintf("SB n 0x%X o 0x%X rle %d ridx %d\n\r",cval,lval,rlecnt,rxbufdidx); 
         check_rle();
         tx_d_samp(d,cval);
         check_tx_buf(TX_BUF_THRESH);
       }//if cval!=lval
       lval=cval;
     }//for s
  check_rle();
  check_tx_buf(1);
}//send_slices_1B

//2B is 9-16 channels
//For all modes the sample bits are always continous/fully packed
void __attribute__ ((noinline)) send_slices_2B(sr_device_t *d,uint8_t *dbuf){
   send_slice_init(d,dbuf);
   send_first_dig_sample(d,dbuf);
   for(int s=0;s<samp_remain;s++){
       cval=(*((uint16_t *) (dbuf+rxbufdidx)));
       rxbufdidx+=2;
       if(cval==lval){
	     rlecnt++;
         }
       else{
          check_rle();
          tx_d_samp(d,cval);
           check_tx_buf(TX_BUF_THRESH);
       }//if cval!=lval
       lval=cval;
     }//for s
   check_rle();
   check_tx_buf(1);
}//send_slices_2B
//4B is 17-21 channels in BASE_MODE
//It is also used with 17-26 channels in DIG_26_MODE, and 17-32 channels in DIG_32_MODE
void __attribute__ ((noinline)) send_slices_4B(sr_device_t *d,uint8_t *dbuf){
   send_slice_init(d,dbuf);
   send_first_dig_sample(d,dbuf);
   for(int s=0;s<samp_remain;s++){
       cval=(*((uint32_t *) (dbuf+rxbufdidx)));
       rxbufdidx+=4;
       //Mask invalid bits, remove 29-31, and shift down 26-28 over 23-25
       #ifdef DIG_26_MODE
        cval=(cval&MEM_D_MASK_L)|((cval&MEM_D_MASK_U)>>3);
       #elif BASE_MODE
        //mask off upper unused
        cval=cval&MEM_D_MASK_L;
        //No change for DIG_32_MODE as all are defined  
       #endif
       if(cval==lval){
      	   rlecnt++;
         }
       else{
         check_rle();
         tx_d_samp(d,cval);
         check_tx_buf(TX_BUF_THRESH);
       }//if cval!=lval
       lval=cval;
     }//for s
   check_rle();
   check_tx_buf(1);
}//send_slices_4B


//Slice transmit code, used for all cases with any analog channels 
//All digital channels for one slice are sent first in 7 bit bytes using values 0x80 to 0xFF
//Analog channels are sent next, with each channel taking one 7 bit byte using values 0x80 to 0xFF.
//This does not support run length encoding because it's not clear how to define RLE on analog signals
//This functional will only be called in BASE_MODE, as neither DIG_26_MODE or DIG_32 mode
//have analog support
uint32_t send_slices_analog(sr_device_t *d,uint8_t *dbuf,uint8_t *abuf){
   send_slice_init(d,dbuf);
   uint32_t lval=0;
   for(int s=0;s<samp_remain;s++){
         if(d->d_mask){
            cval=get_cval(dbuf);
            tx_d_samp(d,cval);
	    //Dprintf("s %d cv %X bps %d idx t %d r %d \n\r",s,cval,d_dma_bps,txbufidx,rxbufdidx);
         }
         for(char i=0;i<d->a_chan_cnt;i++){
           txbuf[txbufidx]=(abuf[rxbufaidx]>>1)|0x80;
           txbufidx++;
           rxbufaidx++;
	   //Dprintf("av %X cnt %d idx t %d r %d\n\r",abuf[rxbufaidx-1],d->a_chan_cnt,txbufidx,rxbufaidx);
         } 
         //Since this doesn't support RLEs we don't need to buffer
         //extra bytes to prevent txbuf overflow, but this value
         //works well anyway
         check_tx_buf(TX_BUF_THRESH);
   }//for s
   check_tx_buf(1);
}//send_slices_analog

//This function monitors the dma interrupt handler outputs to send the remainder of a full DMA buffer.
void send_half(void){
  bool sendlower;
  uint32_t dbuf_start, abuf_start;
  sendlower=(num_halves & 1) ? false : true;
  //return immediately if not in a sending state
  if((dev.state==SENDING)||(dev.state==DMA_DONE))
  {
    sho_cnt++;
  }else{
    return;
  }
  //We have a full DMA buffer, send it.
  if(dma_halves>num_halves){
       tx_cnt++;
       dbuf_start=sendlower ? dev.dbuf0_start : dev.dbuf1_start;
       abuf_start=sendlower ? dev.abuf0_start : dev.abuf1_start;
       //Dprintf("d buffers %d %d %d\n\r",dev.dbuf0_start,dev.dbuf1_start,dbuf_start);
       //Dprintf("a buffers %d %d %d\n\r",dev.abuf0_start,dev.abuf1_start,abuf_start);
       if(dev.a_mask){ send_slices_analog(&dev,&(capture_buf[dbuf_start]),
                                                 &(capture_buf[abuf_start]));}
       else if(d_dma_bps==0){send_slices_D4(&dev,&(capture_buf[dbuf_start]));}
       else if(d_dma_bps==1){send_slices_1B(&dev,&(capture_buf[dbuf_start]));}
       else if(d_dma_bps==2){send_slices_2B(&dev,&(capture_buf[dbuf_start]));}
       else {                send_slices_4B(&dev,&(capture_buf[dbuf_start]));}
        num_halves++;
  }//if dma_halves>num_halves
  //If we ever recieve a usb_plus, consider all samples to be sent, even if not in continuous mode
  if(dev.usb_plus){
    dev.state=SAMPLES_SENT;
 //   Dprintf("SH_USB_PLUS_SS\n\r");
  //At DMA_DONE transition to SAMPLES_SENT when all samples are sent    
  }else if(dev.state==DMA_DONE){
    if((dev.scnt>=dev.num_samples) || (dev.cont==true)){
      dev.state=SAMPLES_SENT;
      Dprintf("SH_SSENT %d %d\n\r",dev.scnt,dev.num_samples);
    }else{
      //Even with dma disabled, we still might have one more half buffer to send
      //so allow this loop to be called again for the remaining half buffer
      if(dma_halves-num_halves==1){
        //Dprintf("ONEMORE\n\r");
      }else{
        if(mask_xfer_err==false){
        //If we have more than one extra we have some kind of overflow/error/abort etc
        //that isn't expected.
        Dprintf("Unexpected state cnt %d %d halves %d %d %d\n\r",dev.scnt,dev.num_samples,exp_halves,dma_halves,num_halves);
        dev.state=ABORTED;
        }
     }//else
    }//DMA_DONE
  }//else
}//send_half

//Handle interrupts generated by ADC or PIO.  If both are enabled they may come in
//either order, so wait for both if only one is seen.
void dma_int_handler(){
  int sts;
  //Have we detected any cases were dma should be turnned off and interrupts disabled?
  //this includes error/abort and non error/abort cases
  bool dma_done=false; 
  bool partial=true;
  sts=dma_hw->ints0;
  currintmask|=sts;
  //We shouldn't reach the interrupt handler until state transitions from
  //started to sending
  if(dev.state==STARTED){
    Dprintf("ERR: int handler at started %X\n\r",dma_hw->ints0);
    dma_hw->ints0=dma_hw->ints0;
    dev.state=ABORTED;
    dma_done=true;
  }
  //If we aren't in sending state then we don't need DMA results
  else if(dev.state!=SENDING){
     Dprintf("INT skip state %d \n\r",dev.state);
     dma_done=true;
  }
  //The "+" from the host ends a transmit, in both continuous and non-continuous modes
  else if(dev.usb_plus){
     Dprintf("INT skip plus\n\r");
     dma_done=true;
  }  
  //This first checks says that if we have seen an IRQ for either of the 
  //lower halves and an IRQ for either of the upper halves that we have overflowed.
  //This is rather exceptional as it means we managed to finish DMAs for both
  //halves and only called the interrupt handler once.
  else if((mask_xfer_err==false)
       && ((currintmask&h0intmask) && (currintmask&h1intmask))){
      Dprintf("Int Overflow0 a %d b %d c %d d %d e %d halves %d %d masks %X %X %X \n\r",
             acnt,bcnt,ccnt,dcnt,ecnt,dma_halves,num_halves,currintmask,h0intmask,h1intmask);
      dma_done=true;
      dev.state=ABORTED;
  }
  //If we haven't detected any errors process the current interrupt mask
  //Note that in very high sample rates with low number of samples we may reach this interrupt handler
  //with both halves being valid, so check and clear each half independently
  else { 
    if((currintmask&h0intmask)==h0intmask){
       //Dprintf("H0\n\r");
       partial=false;
       currintmask=currintmask & ~h0intmask;
       dma_halves++;
       //print_DMA();
    }     
    //We have gotten the expected interrupts for upper
    if((currintmask&h1intmask)==h1intmask){
       //Dprintf("H1\n\r");
       partial=false;
       currintmask=currintmask & ~h1intmask;
       dma_halves++;
       //print_DMA();
     }  
  }
  //A partial means we have gotten either the PIO or the DMA for a given half 
  //but not both, in that case exit the handler waiting for the other to catch up
  if(partial){
     //Dprintf("PRT %X\n",currintmask);
  }
  
//This 2nd overflow check says if dma_halves is more than one ahead of num_halves then we are starting to 
//overwrite a buffer we are sending. Note that it is after we increment dma_halves .
  if((mask_xfer_err==false)
     && (dma_halves-num_halves>1))
   {
    Dprintf("Int Overflow1 a %d b %d c %d d %d e %d halves %d %d masks %X %X %X \n\r",
      acnt,bcnt,ccnt,dcnt,ecnt,dma_halves,num_halves,currintmask,h0intmask,h1intmask);
    dma_done=true;
    dev.state=ABORTED;
   }
   //Stop non continous mode when we reach expected number of halves
   if(dma_halves==exp_halves){
    //Dprintf("EXP_DNE %d\n\r",exp_halves); 
    dma_done=true;
   }
  //Once dma_done is detected disable all the dma channels and then disable
  //the interrupts, and then change the state.  This is not a full
  //cleanup of all state because we still must send the remaining samples
  //and the bytecnt etc, so it's just enough to stop dma transfers and interrupts
  if(dma_done){
     // Dprintf("DMADNE %X\n\r",sts);
      currintmask=0;
      //Aborting a DMA engine can lead to undefined behavior.  Thus the main
      //channels are not aborted as data corruption was seen in some cases.
      //Aborting the maintenance channels is generally safe because they are only one
      //DMA operation and aborting them is sufficient to break the chain and prevent
      //new DMA operations from overwritting data that hasn't been sent across USB.
      //dma_channel_abort(admachan0);
      //dma_channel_abort(admachan1);
      //dma_channel_abort(pdmachan0);
      //dma_channel_abort(pdmachan1);
      dma_channel_abort(amaintchan0);
      dma_channel_abort(amaintchan1);
      dma_channel_abort(pmaintchan0);
      dma_channel_abort(pmaintchan1);
      dma_channel_set_irq0_enabled(admachan0, false);   
      dma_channel_set_irq0_enabled(pdmachan0, false);   
      dma_channel_set_irq0_enabled(admachan1, false);   
      dma_channel_set_irq0_enabled(pdmachan1, false); 
      if((dev.state!=ABORTED)&&(dev.usb_plus==false)){
        dev.state=DMA_DONE;
      } 

  }
  //clear the pended interrupt
  dma_hw->ints0=sts;
}
#ifdef PIN_TEST_MODE
//Force a test pattern to ensure the design is working.  Ideally we could DMA
//from a counter and write to the GPIOs. However, DMA isn't allowed access to SIO.
//We could also use the PIOs, but PIOs are used for the main functionaliy
//and PIO clocks are modified based on sample rate, so using the normal timer ensures
//more predictable timing.
//Note: This function makes no attempt to determine which pins are enabled.
//Note: During transitions between DMA engines the delay between timers may jump from
//100us to ~300us, thus "stretching" the sample values of the test.  Not sure why
//such a large delta exists, but just be aware of the limitation. It may only happen with
//high rates of serial print outs...
bool pin_test_timer_callback(__unused struct repeating_timer *t){
//  Dprintf(".\n\r");
//  pin_test_cnt=pin_test_cnt+0x00100001; //0xF070301;
  pin_test_cnt=pin_test_cnt+0x01001001; //0xF070301;
  if(dev.state==SENDING){
     systick_array[systick_idx++]=time_us_32();
     systick_idx%=128;

  }
  //In case the user forgets to not drive the pins, only drive the test signal when not idle
  //to limit drive fights.
  if(dev.state!=IDLE){
     sio_hw->gpio_out=pin_test_cnt;
                            //mask        value
     gpio_set_dir_masked(PIN_TEST_MASK,PIN_TEST_MASK); //masked set per pin.  1 is output, 0 is input
  }else{
     sio_hw->gpio_out=0;
                       //mask        value
     gpio_set_dir_masked(PIN_TEST_MASK,0); //masked set per pin.  1 is output, 0 is input
  }
  return true;
}
void core1_entry(){
    Dprintf("*********WARNING PIN TEST MODE-DO NOT CONNECT TO PINS\n\r");
    gpio_init_mask(PIN_TEST_MASK); //set to function SIO as input
    //Note the pin directions are handled in the timer call back.
    //gpio_set_dir_masked(PIN_TEST_MASK,PIN_TEST_MASK); //masked set per pin.  1 is output, 0 is input
    //                     delay in us, call back,      userdata, timer    
    add_repeating_timer_us(100,pin_test_timer_callback,NULL,&pt_timer);
    gpio_set_dir_masked(PIN_TEST_MASK,PIN_TEST_MASK); //masked set per pin.  1 is output, 0 is input
    Dprintf("Pin Test Mode Timer Added\n\r");
    while(1){
      //Attempt to sleep the core to reduce contention for the memory bus
      //and maybe save some power
        __wfe();
    }
  }//core1_entry

#endif //PIN_TEST_MODE
int main(){
    int delay=100;
    stdio_usb_init();
    #if (UART_EN == 1)
     uart_set_format(uart0,8,1,0);
     uart_init(uart0,UART_BAUD);
     gpio_set_function(0, GPIO_FUNC_UART);
    //The uart Rx has never been used, but left in for the baseline definition
     gpio_set_function(1, GPIO_FUNC_UART); 
    #endif  
    //This sleep may not be necessary, but was added to give USB extra time to come up.
    //But an extra .1 seconds won't bother anything...
    sleep_us(100000);    
    Dprintf("\n\rHello from PICO sigrok device \n\r");
    //Pulse the LED in a morse code "P" to confirm programming
    #ifdef HAS_LED
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_function(PICO_DEFAULT_LED_PIN,GPIO_FUNC_SIO);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        //More Code "P"
        //dit
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(delay);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(delay);
        //dah
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(delay*3);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(delay);
        //dah
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(delay*3);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(delay);
        //dit
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(delay);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(delay*2);
        
    #endif
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    Dprintf("pll_sys = %dkHz\n\r", f_pll_sys);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    Dprintf("clk_sys = %dkHz\n\r", f_clk_sys);
    #ifndef DIG_32_MODE 
    //Set GPIO23 (TP4) to control switched mode power supply noise
    //This may reduce noise into the ADC in some use cases.   
    gpio_init_mask(1<<23);
    gpio_set_dir_masked(1<<23,1<<23);
    gpio_put_masked(1<<23,1<<23);
    #endif
    //Early CDC IO code had lots of sleep statements, but the TUD code seems to have sufficient
    //checks that this isn't needed, but it doesn't hurt...
    sleep_us(100000);    
    //GPIOs 26 through 28 (the ADC ports) are on the PICO, GPIO29 is not a pin on the PICO
    //Note that digital only modes don't block all configuration related to ADC, but does enough
    //to ensure we can properly sample the pins digitally.
    #ifdef BASE_MODE
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_init();
    #endif

    halves_seen=0;

    admachan0 = dma_claim_unused_channel(true);
    pdmachan0 = dma_claim_unused_channel(true);
    admachan1 = dma_claim_unused_channel(true);
    pdmachan1 = dma_claim_unused_channel(true);
    amaintchan0 = dma_claim_unused_channel(true);
    amaintchan1 = dma_claim_unused_channel(true);
    pmaintchan0 = dma_claim_unused_channel(true);
    pmaintchan1 = dma_claim_unused_channel(true);
    Dprintf("DMA Channels A0 %d P0 %d A1 %d P1 %d M %d %d %d %d\n",
            admachan0,pdmachan0,admachan1,pdmachan1,amaintchan0,amaintchan1,pmaintchan0,pmaintchan1);
    acfg0 = dma_channel_get_default_config(admachan0);
    acfg1 = dma_channel_get_default_config(admachan1);
    pcfg0 = dma_channel_get_default_config(pdmachan0);
    pcfg1 = dma_channel_get_default_config(pdmachan1);
    amcfg0 = dma_channel_get_default_config(amaintchan0);
    amcfg1 = dma_channel_get_default_config(amaintchan1);
    pmcfg0 = dma_channel_get_default_config(pmaintchan0);
    pmcfg1 = dma_channel_get_default_config(pmaintchan1);
    //ADC transfer 1 bytes, PIO transfer the 4B default
    //maintenance transfers 1 32b write pointers
    channel_config_set_transfer_data_size(&acfg0, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&acfg1, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&pcfg0, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&pcfg1, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&amcfg0, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&amcfg1, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&pmcfg0, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&pmcfg1, DMA_SIZE_32);
    //no dmas do read increments
    channel_config_set_read_increment(&acfg0, false);
    channel_config_set_read_increment(&acfg1, false);
    channel_config_set_read_increment(&pcfg0, false);
    channel_config_set_read_increment(&pcfg1, false);
    channel_config_set_read_increment(&amcfg0, false);
    channel_config_set_read_increment(&amcfg1, false);
    channel_config_set_read_increment(&pmcfg0, false);
    channel_config_set_read_increment(&pmcfg1, false);
    //ADC and PIO do write increments, the maintenance don't
    channel_config_set_write_increment(&acfg0, true);
    channel_config_set_write_increment(&acfg1, true);
    channel_config_set_write_increment(&pcfg0, true);
    channel_config_set_write_increment(&pcfg1, true);


    channel_config_set_write_increment(&amcfg0, false);
    channel_config_set_write_increment(&amcfg1, false);
    channel_config_set_write_increment(&pmcfg0, false);
    channel_config_set_write_increment(&pmcfg1, false);

    // Pace transfers based on availability of ADC samples or PIO samples
    channel_config_set_dreq(&acfg0, DREQ_ADC);
    channel_config_set_dreq(&acfg1, DREQ_ADC);
    channel_config_set_dreq(&pcfg0, pio_get_dreq(pio,piosm,false));
    channel_config_set_dreq(&pcfg1, pio_get_dreq(pio,piosm,false));

    //Maintenance transfer once as fast as possible
    //Note that this most likely does not use DMA channel 0, but that's the best
    //value defined by the SDK that is common to rp2040 and rp2350
    channel_config_set_dreq(&amcfg0, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
    channel_config_set_dreq(&amcfg1, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
    channel_config_set_dreq(&pmcfg0, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
    channel_config_set_dreq(&pmcfg1, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);

    //Chaining is always enabled with PIO0->PMAINT0->PIO1->PMAINT1->PIO0
    //and ADMA0->AMAINT0->ADMA1->AMAINT1->ADMA0, except in mask_xfer_err case where 
    //we know we will only have two half buffers and will likely fill them way before 
    //we can process them.  In that case we end the chaining by not doing a chaint_to
    //from maintenance 1.
    //The mask_xfer_err override is handled as part of the init for each sample run
    channel_config_set_chain_to(&acfg0,amaintchan0);
    channel_config_set_chain_to(&amcfg0,admachan1);
    channel_config_set_chain_to(&acfg1,amaintchan1);
    channel_config_set_chain_to(&pcfg0,pmaintchan0);
    channel_config_set_chain_to(&pmcfg0,pdmachan1);
    channel_config_set_chain_to(&pcfg1,pmaintchan1);
    //PIO status
    //TODO (coding style) - use a better csr reference name that points diretly i.e. pio0_hw->ctrl
    volatile uint32_t *pioctrl,*piofstts,*piodbg,*pioflvl;
    pioctrl=(volatile uint32_t *)(PIO0_BASE); //PIO CTRL
    piofstts=(volatile uint32_t *)(PIO0_BASE+0x4); //PIO FSTAT
    piodbg=(volatile uint32_t *)(PIO0_BASE+0x8); //PIO DBG
    pioflvl=(volatile uint32_t *)(PIO0_BASE+0x10); //PIO FLVL
    //TODO (coding style)- use a better csr reference name that points diretly
    volatile uint32_t *pio0sm0clkdiv;
    pio0sm0clkdiv=(volatile uint32_t *)(PIO0_BASE+0xc8); 
    //Give High priority to DMA to ensure we don't overflow the PIO or DMA fifos
    //The DMA controller must read across the common bus to read the PIO fifo so enabled both reads and write
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    
    init(&dev);
    //Since RP2040 is 32 bit this should always be 4B aligned, and it must be because the PIO
    //does DMA on a per byte basis
    //If either malloc fails the code will just hang
    Dprintf("Malloc %d bytes\n\r",DMA_BUF_SIZE);
    capture_buf=malloc(DMA_BUF_SIZE);
    Dprintf("DMA capture buf start %p\n\r",(void *)capture_buf);
    //Ensure we leave 10k or more left for any dynamic allocations that need it
    uint8_t *tptr;
    tptr=malloc(10000);
    Dprintf("10K free start %p\n\r",(void *)tptr);
    free(tptr); 


   gpio_init_mask(GPIO_D_MASK); //set as GPIO_FUNC_SIO and clear output enable
   gpio_set_dir_masked(GPIO_D_MASK,0);  //Set all to input
   //Core1 for PIN_TEST_MODE is called at the end so that gpio inits are done
   //after all of the other previous ones that define them as inputs only.
   //It also helps above Dprintf/UART conflicts between the two cores
   #ifdef PIN_TEST_MODE
       multicore_launch_core1(core1_entry);
       //Avoid UART print conflicts by delaying core0 as core1 starts
       sleep_us(100000);
    #endif //PIN_TEST_MODE


while(1){
          ecnt++;
          if(send_resp){
            acnt++;
            int mylen=strlen(dev.rspstr);
            //Don't mix printf with direct to usb commands
  	        //printf("%s",dev.rspstr);
	          my_stdio_usb_out_chars(dev.rspstr,mylen);
            send_resp=false;
           }
//This testmode forces the device into a capture state without needing
//sigrok cli/pulseview to initiate it
//It must be placed immediately before the dev.sending/started check just below
           if(forced_test_mode_en){
              dev.cont=false;
              //These must start from 0 and go up.
              dev.d_mask=0xF;
              dev.a_mask=0x1;
             //min 5khz sample rate
             //TODO (ADC)-Need to add support for ADC clocking/overclocking 
             //- setting to 1MHZ seems to break if ADC is enabled .
             //To do this likely requires a sysclk boost
              dev.sample_rate=2000000;
              dev.num_samples=2000000;
              dev.scnt=0; //number of samples sent
            
              //Clear this on first past
              forced_test_mode_en=false;
              forced_test_mode_run=true;
              Dprintf("Enter forced test mode %d %X \n\r",dev.a_chan_cnt,dev.a_mask);
              tx_init(&dev);

          }
         if(dev.state==STARTED) {
          bool adc_aborting=false;
          Dprintf("STRTING\n\r");
           //Sample rate must always be even.  Pulseview code enforces this 
           //because it specifies a fixed set of frequencies, but sigrok cli can still odd ones.
           dev.sample_rate>>=1;
           dev.sample_rate<<=1;
           //Adjust up and align to 4 to avoid rounding errors etc
           if(dev.num_samples<16){dev.num_samples=16;}
           dev.num_samples=(dev.num_samples+3)&0xFFFFFFFC;
           //Divide capture buf evenly based on channel enables
           //d_size is aligned to 4 bytes because pio operates on words
           //These are the sizes for each half buffer in bytes
           //Calculate relative size in terms of nibbles which is the smallest unit, thus a_chan_cnt is multiplied by 2
           //Nibble size storage is only allow for D4 mode with no analog channels enabled
           //For instance a D0..D5 with A0 would give 1/2 the storage to digital and 1/2 to analog
           uint32_t d_nibbles,a_nibbles,t_nibbles; //digital, analog and total nibbles
           d_nibbles=dev.d_nps;  //digital is in grous of 4 bits
           //Note that this code only supports 7 bit accurate ADC modes.
           a_nibbles=dev.a_chan_cnt*2; //1 byte per sample 
           t_nibbles=d_nibbles+a_nibbles;
           //total buf size must be a multiple of a_nibbles*2, d_nibbles*8, and t_nibbles so that 
           //division is always in whole samples.
           //Also set a multiple of 32  because the dma buffer is split in half, and
           //the PIO does writes on 4B boundaries, and then a 4x factor for any other size/alignment issues
           uint32_t chunk_size=t_nibbles*32;
           if(a_nibbles) chunk_size*=a_nibbles;
           if(d_nibbles) chunk_size*=d_nibbles;
           uint32_t dig_bytes_per_chunk=chunk_size*d_nibbles/t_nibbles;
           uint32_t dig_samples_per_chunk=(d_nibbles) ? dig_bytes_per_chunk*2/d_nibbles : 0;
           uint32_t chunk_samples=d_nibbles ? dig_samples_per_chunk  : (chunk_size*2)/(a_nibbles);
           //total chunks in entire buffer-round to 2 since we split it in half
           uint32_t buff_chunks=(DMA_BUF_SIZE/chunk_size)&0xFFFFFFFE;
           //round up and force power of two since we cut it in half
           uint32_t chunks_needed=((dev.num_samples/chunk_samples)+2)&0xFFFFFFFE;
	         Dprintf("Initial buf calcs nibbles d %d a %d t %d \n\r",d_nibbles,a_nibbles,t_nibbles);
           Dprintf("chunk size %d(bytes) samples per chunk %d total chunks in both halves %d chunks needed %d\n\r",chunk_size,chunk_samples,buff_chunks,chunks_needed);
           Dprintf("dbytes per chunk %d dig samples per chunk %d\n\r",dig_bytes_per_chunk,dig_samples_per_chunk);
           //If all of the samples we need fit in two half buffers or less then we can mask the error
           //logic that is looking for cases where we didn't send one half buffer to the host before
           //the 2nd buffer ended because we only use each half buffer once.
           mask_xfer_err=false;
           //If requested samples are smaller than the buffer, reduce the size so that the 
           //transfer completes sooner.
           //Also, mask the sending of aborts if the requested number of samples fit into RAM
           //Don't do this in continuous mode as the final size is unknown
           if(dev.cont==false){
               if(buff_chunks>chunks_needed){
                  mask_xfer_err=true;
                  buff_chunks=chunks_needed;
                  //Dprintf("Reduce buf chunks to %d\n\r",buff_chunks);
               }
           }
           //In mask_xfer_err mode we don't want the 2nd half to trigger back to the 1st half
           //and overwrite it's data, so set the maintenace config1's to themselves to disable chaining
           if(mask_xfer_err){
                channel_config_set_chain_to(&amcfg1,amaintchan1);
                channel_config_set_chain_to(&pmcfg1,pmaintchan1);
           }else{
                channel_config_set_chain_to(&amcfg1,admachan0);
                channel_config_set_chain_to(&pmcfg1,pdmachan0);
           }
           //Give dig and analog equal fractions
           //This is the size of each half buffer in bytes
           dev.d_size=(buff_chunks*chunk_size*d_nibbles)/(t_nibbles*2);
           dev.a_size=(buff_chunks*chunk_size*a_nibbles)/(t_nibbles*2);
           dev.samples_per_half=chunk_samples*buff_chunks/2;
           exp_halves=dev.cont ? -1 : dev.num_samples/dev.samples_per_half;
           if(dev.cont==false && (dev.num_samples%dev.samples_per_half)) exp_halves++;
           Dprintf("Final sizes d %d a %d mask err %d samples per half %d exp %d\n\r",dev.d_size,dev.a_size,mask_xfer_err,dev.samples_per_half,exp_halves);

           //Clear any previous ADC over/underflow	    
            volatile uint32_t *adcfcs;
            adcfcs=(volatile uint32_t *)(ADC_BASE+0x8);//ADC FCS
            *adcfcs|=0xC00;
          //Ensure any previous dma is done 
          //The cleanup loop also does this but it doesn't hurt to do it twice
          dma_channel_abort(admachan0);
          dma_channel_abort(admachan1);
          dma_channel_abort(pdmachan0);
          dma_channel_abort(pdmachan1);
          dma_channel_abort(amaintchan0);
          dma_channel_abort(amaintchan1);
          dma_channel_abort(pmaintchan0);
          dma_channel_abort(pmaintchan1);

          dev.dbuf0_start=0;
          bytecnt=0;
          dev.dbuf1_start=dev.d_size;
          dev.abuf0_start=dev.dbuf1_start+dev.d_size;
          dev.abuf1_start=dev.abuf0_start+dev.a_size;
          volatile uint32_t *adcdiv;
          adcdiv=(volatile uint32_t *)(ADC_BASE+0x10);//ADC DIV
          //   Dprintf("adcdiv start %u\n\r",*adcdiv);
	        //	  Dprintf("starting d_nps %u a_chan_cnt %u d_size %u a_size %u a_mask %X\n\r"
          //         ,dev.d_nps,dev.a_chan_cnt,dev.d_size,dev.a_size,dev.a_mask);
          Dprintf("start offsets d0 0x%X d1 0x%X a0 0x%X a1 0x%X samperhalf %u\n\r"
              ,dev.dbuf0_start,dev.dbuf1_start,dev.abuf0_start,dev.abuf1_start,dev.samples_per_half);
//For debug clear out initial values, but not needed in normal operation              
//          for(uint32_t x=0;x<DMA_BUF_SIZE;x++){
//            capture_buf[x]=0x12; 
//          }                  
#ifdef PIN_TEST_MODE
          for(uint32_t x=0;x<SYSTICK_SIZE;x++){
            systick_array[x]=0x0; 
          }         
          systick_idx=0;       
#endif //PIN_TEST_MODE
          //Dprintf("starting data buf values 0x%X 0x%X\n\r",capture_buf[dev.dbuf0_start],capture_buf[dev.dbuf1_start]);
          uint32_t adcdivint=48000000ULL/(dev.sample_rate*dev.a_chan_cnt);
          if(dev.a_chan_cnt){
      	     adc_run(false);
             //             en, dreq_en,dreq_thresh,err_in_fifo,byte_shift to 8 bit
             adc_fifo_setup(false, true,   1,           false,       true); 
             adc_fifo_drain();
             //Dprintf("astart cnt %u div %f\n\r",dev.a_chan_cnt,(float)adcdivint);
             //This sdk function doesn't support support the fractional divisor
             // adc_set_clkdiv((float)(adcdivint-1));
             //The ADC divisor has some not well documented limitations.
             //-A value of 0 actually creates a 500khz sample clock.
             //-Values below 96 are clamped to 96 because a conversion takes a minimum of 96 cycles.
   	         //It is also import to subtract one from the desired divisor
	           //because the period of ADC clock is 1+INT+FRAC/256
	           //For the case of a requested 500khz clock, we would normally write
             //a divisor of 95, but doesn't give the desired result, so we use 
             //the 0 value instead.
             //Fractional divisors should generally be avoided because it creates
             //skew with digital samples.
             uint8_t adc_frac_int;
             adc_frac_int=(uint8_t)(((48000000ULL%dev.sample_rate)*256ULL)/dev.sample_rate);
             if(adcdivint<=96){ 
               Dprintf("adcdivint of %d below 96, aborting\n\r",adcdivint);
               dev.state=ABORTED;
               adc_aborting=true;
               *adcdiv=0;
             }else{ //adcdivint legal
	              *adcdiv=((adcdivint-1)<<8)|adc_frac_int; 
                Dprintf("adcdiv %u frac %d adcdivint %d\n\r",*adcdiv,adc_frac_int,adcdivint);
                //This is needed to clear the AINSEL so that when the round robin arbiter starts 
                //we start sampling on channel 0
                adc_select_input(0);
                adc_set_round_robin(dev.a_mask & 0x7);
                //             en, dreq_en,dreq_thresh,err_in_fifo,byte_shift to 8 bit
                adc_fifo_setup(true, true,   1,           false,       true);
                //set adc0 to immediate trigger (but without adc_run it shouldn't start)
                //adc1 and the maintenance aren't triggered because they are chained to each other
                //                      channel, config, write_addr,                   read_addr,transfer_count,trigger)
                dma_channel_configure(admachan0,&acfg0,&(capture_buf[dev.abuf0_start]),&adc_hw->fifo,dev.a_size,true);
                dma_channel_configure(admachan1,&acfg1,&(capture_buf[dev.abuf1_start]),&adc_hw->fifo,dev.a_size,false);
                //The maintenance DMA for ADC reads the capture_buff offset value and updates the ADC DMAs with it
                amaddrs[0]=(uint32_t *)&capture_buf[dev.abuf0_start];
                amaddrs[1]=(uint32_t *)&capture_buf[dev.abuf1_start];
                //This is about as close to a common/portable address offset definition between devices to
                //find the offset of the write_addr_offset that is non-triggering.
                tmpaddr0=DMA_BASE+DMA_CH0_WRITE_ADDR_OFFSET+(DMA_CH1_READ_ADDR_OFFSET*admachan0);
                tmpaddr1=DMA_BASE+DMA_CH0_WRITE_ADDR_OFFSET+(DMA_CH1_READ_ADDR_OFFSET*admachan1);
                //Dprintf("ADMA Maint %X %X %X %X %X %X\n\r",amaddrs[0],amaddrs[1],tmpaddr0,tmpaddr1,&amaddrs[0],&amaddrs[1]);
                //                      channel, config, write_addr,            read_addr,transfer_count,trigger)
                dma_channel_configure(amaintchan0,&amcfg0, (uint32_t *) tmpaddr0,&amaddrs[0]  ,1,false);
                dma_channel_configure(amaintchan1,&amcfg1, (uint32_t *) tmpaddr1,&amaddrs[1]  ,1,false);
                adc_fifo_drain();
              } //adcdivint legal
          }//any analog enabled
          if(dev.d_mask){
             //analyzer_init from pico-examples
             //Due to how PIO shifts in bits, if any digital channel within a group of 8 is set, 
             //then all groups below it must also be set. We further restrict it in the tx_init function
             //by saying digital channel usage must be continous.
 /* pin count is restricted to 4,8,16 or 32, and pin count of 4 is only used
Pin count is kept to a powers of 2 so that we always read a sample with a single byte/word/dword read
for faster parsing.  
   if analog is disabled and we are in D4 mode
    bits d_dma_bps   d_tx_bps
    0-4    0          1        No analog channels
    0-4    1          1        1 or more analog channels
    5-7    1          1
    8      1          2
    9-12   2          2
    13-14  2          2
    15-16  2          3
    17-21  4          3
*/
             dev.pin_count=0 ;
             if(dev.d_mask&0x0000000F) dev.pin_count+=4;
             if(dev.d_mask&0x000000F0) dev.pin_count+=4;
             if(dev.d_mask&0x0000FF00) dev.pin_count+=8;
             if(dev.d_mask&0x0FFF0000) dev.pin_count+=16;
             //If 4 or less channels are enabled but ADC is also enabled, set a minimum size of 1B of PIO storage
             if((dev.pin_count==4)&&(dev.a_chan_cnt)){dev.pin_count=8;}
             d_dma_bps=dev.pin_count>>3;
             //Dprintf("pin_count %d\n\r",dev.pin_count);
             uint16_t capture_prog_instr;
             capture_prog_instr = pio_encode_in(pio_pins, dev.pin_count);
             //Dprintf("capture_prog_instr 0x%X\n\r",capture_prog_instr);
             struct pio_program capture_prog = {
                .instructions = &capture_prog_instr,
                .length = 1,
                .origin = -1
             };
             uint offset = pio_add_program(pio, &capture_prog);
             // Configure state machine to loop over this `in` instruction forever,
             // with autopush enabled.
             pio_sm_config c = pio_get_default_sm_config();
             #ifdef DIG_26_MODE
               sm_config_set_in_pins(&c, 0); //start at GPIO0 since uart isn't used
             #elif DIG_32_MODE
               sm_config_set_in_pins(&c, 0); //start at GPIO0 since uart isn't used
             #else 
               sm_config_set_in_pins(&c, 2); //start at GPIO2 (keep 0 and 1 for uart)
             #endif
             sm_config_set_wrap(&c, offset, offset);
             uint16_t div_int;              
             uint8_t frac_int;
             div_int=frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000/dev.sample_rate;
             if(div_int<1) div_int=1;
             frac_int=(uint8_t)(((frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000%dev.sample_rate)*256ULL)/dev.sample_rate);
	           Dprintf("PIO sample clk %u divint %d divfrac %d \n\r",dev.sample_rate,div_int,frac_int);
             //Unlike the ADC, the PIO int divisor does not have to subtract 1.
             //Frequency=sysclkfreq/(CLKDIV_INT+CLKDIV_FRAC/256)
             sm_config_set_clkdiv_int_frac(&c,div_int,frac_int);

             //Since we enable digital channels in groups of 4, we always get 32 bit words
             sm_config_set_in_shift(&c, true, true, 32);
             sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
             pio_sm_init(pio, piosm, offset, &c);
             //Analyzer arm from pico examples
             pio_sm_set_enabled(pio, piosm, false); //clear the enabled bit
             //XOR the shiftctrl field with PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
             //Do it twice to restore the value
             pio_sm_clear_fifos(pio, piosm); 
             //write the restart bit of PIO_CTRL
             pio_sm_restart(pio, piosm);
             //Since PIO transfers 32 bit values but DMA transfers 8, the d_size is divided by 4.
             //Dprintf("DMABufCfg d0_start %d d1_start %d d_size %d\n\r",dev.dbuf0_start,dev.dbuf1_start,dev.d_size);
	           //                    number    config   buffer target                  piosm          xfer size  trigger
             dma_channel_configure(pdmachan0,&pcfg0,&(capture_buf[dev.dbuf0_start]),&pio->rxf[piosm],dev.d_size>>2,true);
             dma_channel_configure(pdmachan1,&pcfg1,&(capture_buf[dev.dbuf1_start]),&pio->rxf[piosm],dev.d_size>>2,false);
             //The maintenance DMA for PIO reads the capture_buff offset value and updates the ADC DMAs with it
             pmaddrs[0]=(uint32_t *)&capture_buf[dev.dbuf0_start];
             pmaddrs[1]=(uint32_t *)&capture_buf[dev.dbuf1_start];
             //This is about as close to a common/portable address offset definition between devices to
             //find the offset of the write_addr_offset that is non-triggering.
             tmpaddr0=DMA_BASE+DMA_CH0_WRITE_ADDR_OFFSET+(DMA_CH1_READ_ADDR_OFFSET*pdmachan0);
             tmpaddr1=DMA_BASE+DMA_CH0_WRITE_ADDR_OFFSET+(DMA_CH1_READ_ADDR_OFFSET*pdmachan1);
             //Dprintf("PDMA Maint %X %X %X %X %X %X\n\r",pmaddrs[0],pmaddrs[1],tmpaddr0,tmpaddr1,&pmaddrs[0],&pmaddrs[1]);
             //                      channel, config, write_addr, read_addr,transfer_count,trigger)
             dma_channel_configure(pmaintchan0,&pmcfg0, (uint32_t *)tmpaddr0,&pmaddrs[0]  ,1,false);
             dma_channel_configure(pmaintchan1,&pmcfg1, (uint32_t *)tmpaddr1,&pmaddrs[1]  ,1,false);

             } //if dev.d_mask
          //Dprintf("LVL0mask 0x%X\n\r",dev.lvl0mask);
          //Dprintf("LVL1mask 0x%X\n\r",dev.lvl1mask);
          //Dprintf("risemask 0x%X\n\r",dev.risemask);
          //Dprintf("fallmask 0x%X\n\r",dev.fallmask);
          //Dprintf("edgemask 0x%X\n\r",dev.chgmask);

          //Dprintf("capture_buf base %p \n\r",capture_buf);
          //Dprintf("capture_buf dig %p %p \n\r",&(capture_buf[dev.dbuf0_start]),&(capture_buf[dev.dbuf1_start]));
          //Dprintf("capture_buf analog %p %p\n\r",&(capture_buf[dev.abuf0_start]),&(capture_buf[dev.abuf1_start]));
          //Dprintf("PIOSMCLKDIV 0x%X\n\r",*pio0sm0clkdiv);

          //Dprintf("PIO ctrl 0x%X fstts 0x%X dbg 0x%X lvl 0x%X\n\r",*pioctrl,*piofstts,*piodbg,*pioflvl);
          //Dprintf("DMA channel assignments a %d %d d %d %d\n\r",admachan0,admachan1,pdmachan0,pdmachan1);
          if(!adc_aborting){
          //Clear any pending interrupts
          //All dma interrupts go through a common handler so that we can check for
          //overflows etc.
          dma_channel_set_irq0_enabled(admachan0, true);   
          dma_channel_set_irq0_enabled(pdmachan0, true);   
          dma_channel_set_irq0_enabled(admachan1, true);   
          dma_channel_set_irq0_enabled(pdmachan1, true);   

          h0intmask=0;
          h1intmask=0;
          currintmask=0;

          if(dev.d_mask){
            h0intmask|=1<<pdmachan0;
            h1intmask|=1<<pdmachan1;
          }
          if(dev.a_chan_cnt){
            h0intmask|=1<<admachan0;
            h1intmask|=1<<admachan1;
          }
          dma_halves=0;
          num_halves=0;
          sho_cnt=0;
          tx_cnt=0;
          acnt=0;
          bcnt=0;
          bytecnt=0;
          dcnt=0;
          ecnt=0;

          //Dprintf("XY %X %X %X %X\n",dev.d_mask,dev.a_chan_cnt,h0intmask,h1intmask);
          //Enable logic and analog close together for best possible alignment
  	      //warning - do not put printfs after this line as they will corrupt time measurement and sample start
	        tstart=time_us_32();
          dev.state=SENDING;

          //Pending Interrupts must be cleared immediately before enabling the IRQ
          dma_hw->ints0=dma_hw->ints0;
          irq_set_enabled(DMA_IRQ_0, true);
          irq_set_exclusive_handler(DMA_IRQ_0, dma_int_handler);

          #ifdef BASE_MODE
          adc_run(true); //enable free run sample mode
          #endif
          pio_sm_set_enabled(pio, piosm, true);           
        } //if ~adcaborting
        }//if dev.sending and not started
   //Send sample data
   send_half();
   //Drain all uart rxs (only tx is used for debug) if uart rx is not drained
   //it can cause code in the sdk to lock up serial CDC. These are rare noise/reset events
   //and thus not checked when dev.started to ensure the maintenance loop runs as fast
   //as possible. 
   #if (UART_EN == 1)
   if(dev.state==IDLE){
      while (uart_is_readable_within_us(uart0, 0)) {
            uartch = uart_getc(uart0);
            Dprintf("Uart Char %d\n\r",uartch);
       }
      }
   #endif
   //look for commands on usb cdc 
   ccnt=time_us_32();
   usbintin=getchar_timeout_us(0);
   dcnt=time_us_32();
   //The '+' is the only character we track during normal sampling because it can end
   //a continuous trace or an aborted condition.  
   //A reset '*' should only be seen after we have completed normally or hit an error condition. 
   //The plus ends an aborted loop, is ignored by IDLE, and sends started to IDLE.
   //In all other cases the effect is not immediate and it's up to the interrupt handler
   //or send_half to make use of it.
   if(usbintin=='+'){
       //Dprintf("USB plus\n\r");
       if(dev.state==ABORTED){
        //Clear abort so we stop sending "!"
        Dprintf("Plus ends abort\n\r");
        dev.state==IDLE;
       }else if(dev.state==IDLE){
        Dprintf("Plus in idle ignored\n\r");
       }else if(dev.state==STARTED){
        Dprintf("Plus ends started");
        dev.state==IDLE;
       }else{
        Dprintf("usb_plus set\n\r");
        dev.usb_plus=true;
       }
    }
    //send_resp is set to true but not processed immediately so we can get back to send_half ASAP
    else if(usbintin>=0){  
           bcnt++;
           if(process_char(&dev,(char)usbintin))
             {send_resp=true;} 
    }

	//The libsigrok processing of aborts is not clean, and may try to report the "!" as a bad rle value.
  //Emperically it seems that waiting for a long delay to ensure the host has drained it's Rx buffere
  //and then sending 3 "!" is the most reliable way to get it to exit.
  //As an additional failsafe, a final bytecnt with a count of 0 is sent.
  //In forced test mode we also don't get the usb plus, so the forced exit on abort covers that as well
	if(dev.state==ABORTED){
 	  Dprintf("sending abort! ftm %d num_halves %d dma_halves %d sho cnt %d tx_cnt %d\n\r",
            forced_test_mode_run,num_halves,dma_halves,sho_cnt,tx_cnt);
    sleep_ms(1000);
	  my_stdio_usb_out_chars("!!!",3);
    sleep_ms(1000);
    my_stdio_usb_out_chars("$0+",3);
    sleep_ms(1000);
    dev.state=IDLE;
   }
   //Once we reach SAMPLES_SENT, send the final byte count to ensure no bytes were lost
   if(dev.state==SAMPLES_SENT){
        //The end of sequence byte_cnt uses a "$<byte_cnt>+" format.
        char brsp[16];
        //Give the host time to finish processing samples so that the bytecnt 
        //isn't dropped on the wire
        sleep_us(10000);
        Dprintf("Cleanup bytecnt %d\n\r",bytecnt);
        sprintf(brsp,"$%d%c",bytecnt,'+');
        puts_raw(brsp);
        //Print out debug information after completing, rather than before so that it doesn't 
        //delay the start of a capture
        Dprintf("Complete: SRate %d NSmp %d NHalves %d\n\r",dev.sample_rate,dev.num_samples,num_halves);
        Dprintf("Cont %d bcnt %d\n\r",dev.cont,bytecnt);
        Dprintf("DMsk 0x%X AMsk 0x%X\n\r",dev.d_mask,dev.a_mask);
        Dprintf("Half buffers exp %d DMA %d Sent %d sampperhalf %d\n\r",exp_halves,dma_halves,num_halves,dev.samples_per_half);
        dev.state=IDLE;
#ifdef PIN_TEST_MODE        
        for(int y=0;y<SYSTICK_PRINT;y++){
             int delta;
             if(y==0){delta=0;}
             else if(systick_array[y]==0){delta=0;}
             else{delta=systick_array[y]-systick_array[y-1];}
             //The callback should be every 100us, so print out any large anomolies
             //as a warning that the generated pattern may be stretched.
             if(delta>120){
             Dprintf("*****Systick Anomoly ST%3d %d %d\n\r***",y,systick_array[y],delta);
             }
        }
#endif //PIN_TEST_MODE
    }
    //Since there are so many FSM arcs, it's safest to just continually clear state
    //if we aren't sending
    if(dev.state==IDLE){
     //forced_test_mode is really a one shot deal as there is no way to restart it.
     //Exit the mode so that host accesses will work normally
     forced_test_mode_run=false;
     #ifdef BASE_MODE
     adc_run(false);
     adc_fifo_drain();
     #endif
     pio_sm_restart(pio, piosm);
     pio_sm_set_enabled(pio, piosm, false);
     pio_sm_clear_fifos(pio, piosm);
     pio_clear_instruction_memory(pio);
     dma_channel_abort(admachan0);
     dma_channel_abort(admachan1);
     dma_channel_abort(pdmachan0);
     dma_channel_abort(pdmachan1);
     dma_channel_abort(amaintchan0);
     dma_channel_abort(amaintchan1);
     dma_channel_abort(pmaintchan0);
     dma_channel_abort(pmaintchan1);
     dma_channel_set_irq0_enabled(admachan0, false);   
     dma_channel_set_irq0_enabled(pdmachan0, false);   
     dma_channel_set_irq0_enabled(admachan1, false);   
     dma_channel_set_irq0_enabled(pdmachan1, false); 
     //clear any pended dma interrupts
     dma_hw->ints0=dma_hw->ints0;
     irq_set_enabled(DMA_IRQ_0, false);

     num_halves=0;
     dma_halves=0;
     exp_halves=0;
     currintmask=0;
     dev.usb_plus=false;
  } //if IDLE
}//while(1)
 

}//main
//Depracated trigger logic
//This HW based trigger which should be part of send slices was tested enough to confirm the 
//trigger value worked, however it
//was not fully implemented because the RP2040 wasn't able to perform the trigger detection and 
//memory buffer management to support sample rates that were substantially higher than the 
//stream rates across USB.  Thus there wasn't a compelling reason to have it.
//It's left as an example as to how the masks could be used.
//  To fully support a HW based triggering, a precapture ring buffer of both digital and analog samples
//would need to be created an managed to store and send pretrigger values. 
//The ring buffer would need to support RLEs and would need to ensure it was sent before sending
//other samples capture by the DMA after the trigger event. 
/*
//   uint32_t tlval; Trigger last val
//   tlval=d->tlval;
//   uint32_t all_mask=d->lvl0mask | d->lvl1mask| d->risemask | d->fallmask | d->chgmask;


       if(d->triggered==false) {
         uint32_t matches=0;
         matches|=(~cval & d->lvl0mask);
         matches|=(cval & d->lvl1mask);
         if(d->notfirst){
           matches|=(cval & ~tlval & d->risemask);
           matches|=(~cval & tlval & d->fallmask);
           matches|=(~cval & tlval & d->chgmask);
         }
         if(matches==all_mask){
	   //Dprintf("Triggered c 0x%X l 0x%X \n\r",cval,tlval);
           d->triggered=true;
           //This sends the last val on a trigger because SW based trigger on the host needs to see its
           //value so that rising/falling/edge triggeers will fire there too.
           lbyte=0;
           for(char b=0;b < d->d_tx_bps;b++){
              cbyte=tlval&0xFF;
              txbuf[txbufidx]=(cbyte<<b)|lbyte|0x80;
              lbyte=cbyte>>(7-b);
              tlval>>=8;
	      txbufidx++;
           } //for b          
         }//matches==all_mask
         d->notfirst=true;
       }
       if(d->triggered){ 
             //Transmit samples if we have already triggered.
        }

//save trigger last value to support rising/falling/change values
//      tlval=lval;
End of depracated trigger logic
*/
