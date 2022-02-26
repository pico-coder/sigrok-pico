//Raspberry Pi PICO/RP2040 code to implement a logic analyzer and oscilloscope

/**
 * Some Original code from the pico examples project:
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h" //uart definitions
#include <stdlib.h> //atoi,atol
#include "hardware/gpio.h"
//Note: Hardware libraries must be added to CMakelists.txt
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/pwm.h"
#include "hardware/sync.h" //wev

#include "pico/binary_info.h"
#include "stdarg.h"
#include "pico/multicore.h"
#include <string.h>
#include "sr_device.h"
#include "hardware/clocks.h"

#include "tusb.h"//.tud_cdc_write...


//NODMA is a debug mode that disables the DMA engine and prints raw PIO FIFO outputs
//it is limited to testing a small number of samples equal to the PIO FIFO depths
//#define NODMA 1

//COPY_SER keeps a copy of the serial buffer output, useful for comparing what the
//device sent and the host received
//This should only be used for testing behaviour of captures less than SERBUF_SIZE 
//#define COPY_SER 1
#ifdef COPY_SER
#define SERBUF_SIZE 1000
uint8_t serbuf[SERBUF_SIZE];
uint32_t serbufidx;
#endif

//Timestamps measure time in microsends over various parts of the loop for performance benchmarking
//and then print them out at the end of the run.
//It also keeps track of the size of the serial output buffer which can be key to continuous 
//streaming performance
//#define TIME_STAMPS 1
#ifdef TIME_STAMPS
uint32_t lenh[64];
uint32_t t1[64];
uint32_t t2[64];
uint32_t t3[64];
uint32_t ttl[64];
uint32_t tidx,ttlidx;
#endif


//For debug, save off the sampled data in the regular (non D4 optimized mode)
//User must ensure that the number of requested samples is less than COPY_BUFF_SIZE
//so only use for very small numbers of request samples, and not in continuous mode
//#define COPY_BUFFS 1
#ifdef COPY_BUFFS
#define COPY_BUFF_SIZE 1000
uint8_t cbuf[COPY_BUFF_SIZE*2];
uint32_t cbufidx;
#endif

//These two enable debug print outs of D4 generation, D4_DBG2 are consider higher verbosity
//#define D4_DBG 1
//#define D4_DBG2 2
//#define SLICE_DBG

//Storage size of the DMA buffer.  The buffer is split into two halves so that when the first
//buffer fills we can send the trace data serially while the other buffer is DMA'dinto
#define DMA_BUF_SIZE 200000
uint8_t *capture_buf;
volatile uint32_t c1cnt=0;
volatile uint32_t c0cnt=0;
sr_device_t dev;
volatile uint32_t tstart;
volatile bool send_resp=false;
//The size of the buffer sent to the CDC serial
//Support up to 512 byes plus a few extra for cases where we send a few intial bytes etc.
//For D4 optimized mode we flush at about every 20-32 characters and don't allow large RLE amounts to accumulate
//so this is far more than enough.
//Non D4 modes also typically push when <64 bytes are accumulated so it's likely oversized in all cases.
uint8_t txbuf[516];

//Save off sample data in the D4 optimized flows
//#define D4_COPY_BUFFS
#ifdef D4_COPY_BUFFS
uint32_t d4cw[256];
uint32_t d4cwidx=0,d4cwcnt=0;
#endif
uint32_t num_halves; //track the number of halves we have processed

//Note, the use of not_in_flash_func was an attempt to possibly reduce cases where in continuous
//mode the device would abort when the sample rate was too high.  With a fixed sample rate the aborts
//were not predictable and it was thought that maybe the XIP cache was being overwritten.
//However, the not_in_flash_func didn't noticeably help the problem, but didn't hurt it either.
//The sporadic nature of the aborts could be variability in the host pulling data over usb..


//The function stdio_usb_out_chars is part of the PICO sdk usb library.
//However the function is not externally visible from the library and rather than
//figure out the build dependencies to do that it is just copied here.
//This is much faster than a printf of the string, and even faster than the puts_raw
//supported by the PICO SDK stdio library, which doesn't allow the known length of the buffer
//to be specified. (The C standard write function doesn't seem to work at all).
//This function also avoids the inserting of CR/LF in certain modes.
//The tud_cdc_write_available function returns 256, and thus we have a 256B buffer to feed into
//but the CDC serial issues in groups of 64B.  We send chunks of data in the 16-64B range.
//Smaller than that and we tend to not fully send 64B payloads, and larger we tend to be stuck waiting
//for buffer space when we could be processing more samples.

void __not_in_flash_func(my_stdio_usb_out_chars)(const char *buf, int length) {
    static uint64_t last_avail_time;
    uint32_t owner;
    if (tud_cdc_connected()) {
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
                if (!tud_cdc_connected() ||
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

//This is a simple maintenance loop to monitor serial activity so that core0 can be dedicated
//to monitoring DMA activity and sending trace data.
//Most of the time this loop is stalled with wfes (wait for events).
void __not_in_flash_func(core1_code)(){
   uint32_t testinc=0x0;
   int intin;
   uint8_t uartch;
   uint32_t ctime;
   uint32_t cval;
   while(true){
     //The wait for event (wfe) puts core1 in an idle state
     //Each core instruction takes a memory cycle, as does each core memory or IO register read.
     //The memory fabric supports up to 4 reads per cycle if there are no bank conflicts
     //The DMA engine needs a read of the PIO and ADC and a write of memory
     //and thus chances of conflicts are high.  Thus once we are in sampling mode slow down C1 to allow
     //C0 to loop faster and process faster.
     //Without wfes C1 typically completes 10x the number of outer loops as C0, but with the wfes
     //C0 completes more loops and C1 activity drops to 1% .   
     //Much of the C0 code has built in sev (Send Event) but we also add an explicit one in the main while loop

     if(dev.started){
        c1cnt++;
	__wfe();
        __wfe();
        __wfe();
        __wfe(); 
	__wfe();
	__wfe();
        __wfe();
        __wfe();
        __wfe(); 
	__wfe();
     }     
     if(dev.started==false){
      //always drain all defined uarts as if that is not done it can 
       //effect the usb serial CDC stability
       //these are generally rare events caused by noise/reset events
       //and thus not checked when dev.started 
       while (uart_is_readable_within_us(uart0, 0)) {
            uartch = uart_getc(uart0);
       }
     }    
     //look for commands on usb cdc 
     intin=getchar_timeout_us(0);
     //The '+' is the only character we track during normal sampling because it can end
     //a continuous trace.  A reset '*' should only be seen after we have completed normally
     //or hit an error condition. 
     if(intin=='+'){
       dev.sending=false;
       dev.aborted=false; //clear the abort so we stop sending !!
     }
     //send_resp is used to eliminate all prints from core1 to prevent collisions with 
     //prints in core0
     else if(intin>=0){  
           if(process_char(&dev,(char)intin))
             {send_resp=true;} 
       }
   }//while true

}

//This is an optimized transmit of trace data for configurations with 4 or fewer digital channels 
//and no analog.  Run length encoding (RLE) is used to send counts of repeated values to effeciently utilize 
//USB CDC link bandwidth.
//Samples from PIO are dma'd in 32 bit words, each containing 8 samples of 4 bits (1 nibble).
//Encoding:
//Values 0x80-0xFF encode an rle cnt of a previous value with a new value:
//  Bit 7 is 1 to distinguish from the rle only values.
//  Bits 6:4 indicate a run length up to 7 cycles of the previous value
//  Bits 3:0 are the new value.
//For longer runs, an RLE only encoding uses decimal values 48 to 127 (0x30 to 0x7F)
//as x8 run length values of 8..640.
//The end of sequence byte_cnt uses the normal "$<byte_cnt>+" format.
//The normal abort character "!" is also used if buffer overflows are detected. 
//All other ascii values are reserved.
uint32_t __not_in_flash_func(send_slices_D4)(sr_device_t *d,uint8_t *dbuf){
   uint32_t ccnt=0; //count of characters sent serially
   #ifdef TIME_STAMPS
   uint32_t ts1,ts2,ts3,ts4;
   #endif
   uint8_t nibcurr,niblast;
   uint32_t rlecnt; //count of samples where value hasn't changed
   uint32_t txbufidx=0;
   uint32_t rxbufidx;
   uint32_t samp_remain;
   uint32_t samp_func;  //total samples to be returned for the function call
   uint32_t cword,lword; //current and last word
   uint32_t *cptr;
   //uint32_t total;
   //rlesum/chgcnt/total are very useful in confirming rle calculations are correct but 
   //add needless computes ...
   //uint32_t rlesum=0; //crosscheck of total rles sent
   //uint32_t chngcnt=0; //sum of changes in values plus rlesum (chngcnt+rlesum) must equal num samples

   //Don't optimize the first word (eight samples) perfectly, just send them to make the for loop easier, 
   //and setup the initial conditions for rle tracking
   cptr=(uint32_t *) &(dbuf[0]);
   cword=*cptr;
   #ifdef D4_DBG
   Dprintf("Dbuf %p cptr %p data 0x%X\n\r",(void *)&(dbuf[0]),(void *) cptr,cword);
   #endif
   #ifdef D4_COPY_BUFFS
   if(d4cwidx==0){
     d4cw[0]=cword;
     d4cwidx++;
     d4cwcnt++;
   }
   #endif
   lword=cword;
   for (int j=0;j<8;j++){
     nibcurr=cword&0xF;
     txbuf[j]=(nibcurr)|0x80;
     cword>>=4;
   }
   niblast=nibcurr;      
   cptr=(uint32_t *) &(txbuf[0]);
   txbufidx=8;
   rxbufidx=4;
   rlecnt=0;

   if(d->samples_per_half<=8){
     my_stdio_usb_out_chars(txbuf,txbufidx);
     d->scnt+=d->samples_per_half;
     return txbufidx;
   }
   //chngcnt=8;
   //The total number of 4 bit samples remaining to process from this half.
   //Subtract 8 because we procesed the word above.
   samp_remain=d->samples_per_half-8;

   //If in fixed sample (non-continous mode) only send the amount of samples requested
   if((d->cont==false)&&((d->scnt+samp_remain)>(d->num_samples))){
        samp_remain=d->num_samples-d->scnt;
        d->scnt+=samp_remain;
   }else{
        d->scnt+=d->samples_per_half;
   }
   samp_func=samp_remain+8;
   #ifdef TIME_STAMPS
   ts1=time_us_32();
   #endif
   //Process one  word (8 samples) at a time.
   for(int i=0;i<(samp_remain>>3);i++) {
       cptr=(uint32_t *) &(dbuf[rxbufidx]);
       cword=*cptr;
       rxbufidx+=4;
       #ifdef D4_DBG2
       Dprintf("dbuf0 %p dbufr %p cptr %p \n\r",dbuf,&(dbuf[rxbufidx]),cptr);
       #endif
       #ifdef D4_COPY_BUFFS
       if(d4cwidx<256){
         d4cw[d4cwidx++]=cword;
       }
       d4cwcnt++;
       #endif
       //Send maximal RLE counts in this outer section to the txbuf, and if we accumulate a few of them
       //push to the device so that we don't accumulate large numbers
       //of unsent RLEs.  That allows the host to process them gradually rather than in a flood
       //when we get a value change.
       while(rlecnt>=640){
         txbuf[txbufidx++]=127;
         rlecnt-=640;
         //rlesum+=640;
         if(txbufidx>3){   
            my_stdio_usb_out_chars(txbuf,txbufidx);
            ccnt+=txbufidx;
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
        Dprintf("cword 0x%X nibcurr 0x%X i %d rx idx %u  rlecnt %u \n\r",cword,nibcurr,i,rxbufidx,rlecnt);
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
               //rlesum+=rlemid;
               txbuf[txbufidx++]=(rlemid>>3)+47;
            } 
            //And finally the 0..7 rle along with the new value
            rlecnt&=0x7;
            //rlesum+=rlecnt;
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
       Dprintf("i %d rx idx %u  rlecnt %u \n\r",i,rxbufidx,rlecnt);
       Dprintf("i %u tx idx %d bufs 0x%X 0x%X 0x%X\n\r",i,txbufidx,txbuf[txbufidx-3],txbuf[txbufidx-2],txbuf[txbufidx-1]);
       #endif
       //Emperically found that transmitting groups of around 32B gives optimum bandwidth
       if(txbufidx>=64){
         my_stdio_usb_out_chars(txbuf,txbufidx);
         #ifdef COPY_SER
         int z;
         for(z=0;(z<txbufidx)&&(z+serbufidx<SERBUF_SIZE);z++){
           serbuf[z+serbufidx]=txbuf[z];
         }
         serbufidx+=z;
         if(serbufidx>=SERBUF_SIZE){Dprintf("Serr1 ERROR %d\n\r",serbufidx);}
         #endif   
         ccnt+=txbufidx;
         txbufidx=0;
       }
    }//for i in samp_send>>3
    //At the end of processing the half send any residual samples as we don't maintain state between the halves
    //Maximal 640 values first
    while(rlecnt>=640){
      txbuf[txbufidx++]=127;
      rlecnt-=640;
      //rlesum+=640;
    }
    //Middle rles 8..632
    if(rlecnt>7) {
      int rleend=rlecnt&0x3F8;
      //rlesum+=rleend;
      txbuf[txbufidx++]=(rleend>>3)+47;
    }
    //1..7 RLE 
    //The rle and value encoding counts as both a sample count of rle and a new sample
    //thus we must decrement rlecnt by 1 and resend the current value which will match the previous values
    //(if the current value didn't match, the rlecnt would be 0).
    if(rlecnt){
      rlecnt&=0x7;
      //rlesum+=rlecnt;
      rlecnt--;
      txbuf[txbufidx++]=0x80|nibcurr|rlecnt<<4;
      rlecnt=0;
    }
    #ifdef TIME_STAMPS
    ts2=time_us_32();
    #endif     
    if(txbufidx){
       my_stdio_usb_out_chars(txbuf,txbufidx);
       #ifdef COPY_SER
       int z;
         for(z=0;(z<txbufidx)&&(z+serbufidx<SERBUF_SIZE);z++){
           serbuf[z+serbufidx]=txbuf[z];
         }
         serbufidx+=z;
         if(serbufidx>=SERBUF_SIZE){Dprintf("Serr2 ERROR %d\n\r",serbufidx);}
       #endif   
       ccnt+=txbufidx;
       txbufidx=0;
    }
    #ifdef TIME_STAMPS
    ts3=time_us_32();
    t1[tidx]=ts2-ts1;
    t2[tidx]=ts3-ts2;
    t3[tidx]=txbufidx;
    lenh[tidx]=samp_remain;
    tidx=(tidx+1)%64;
    #endif

    //This is very useful in rle validation but adds extra unneeded computes.
    //Also, the total comparison is only valid in cases where num_samples%8==0.
    //If num_samples%8!=0 the per word sending of samples also issues samples in groups of 8.
    //total=chngcnt+rlesum+rlecnt;
    //if(total!=samp_func){
    //   Dprintf("ERROR: Final chngcnt %d rlesum %d rlecnt %d total %d samp_func %d\n\r",chngcnt,rlesum,rlecnt,total,samp_func);
    //   d->aborted==true;
    //}

    return ccnt;
}//send_slices_D4

//General use slice transmit code, used for all cases with more than 4 digital channels or if any analog
//channels are enabled.
//All digital channels for one slice are sent first in 7 bit bytes using values 0x80 to 0xFF
//Analog channels are sent next, with each channel taking one 7 bit byte using values 0x80 to 0xFF.
//The end of sequence byte_cnt uses the  "$<byte_cnt>+" format.
//The normal abort character "!" is also used if buffer overflows are detected. 
//All other ascii values are reserved.
//This does not support run length encoding because:
//1) It wasn't clear how to do RLE on analog especially if the lsbs happened to toggle.
//2) It's expected (hoped?) that wider busses or analog signals run relatively slower and/or can
//rely on fixed capture.
//3) I ran out of time....

uint32_t __not_in_flash_func(send_slices)(sr_device_t *d,uint8_t *dbuf,uint8_t *abuf){
   uint32_t ts1,ts2,ts3,ts4;
   uint32_t ccnt=0; //count of characters sent serially
   uint32_t txbufidx=0,rxbufdidx=0,rxbufaidx=0;
   uint32_t samp_send;
   uint32_t samp_remain;
   uint8_t lbyte,cbyte; //last and current byte info
   uint8_t bytes_per_slice; //transmit bytes per slice
   uint8_t d_dma_bps; //number of bytes of digital sample data to use
   //Number of bytes to transmit on USB CDC.  
   //Empirically 16-20 seems to give the best bandwidth.
   uint16_t bytes_per_tx=16;


   uint8_t second_nibble=0; //for 4 channel mode are we first or 2nd nibble
   //pin count is restricted to 4,8,16 or 32, and pin count of 4 is used if analog is enabled
   //Thus 0-4 channels have a d_dma_bps of 0 which is special cases in a few places
/* bits d_dma_nps(bps) d_tx_bps
    0-4    1(.5/0)     1
    5-7    2(1)        1
    8      2(1)        2
    9-12   4(2)        2
    13-14  4(2)        2
    15-16  4(2)        3
    17-21  8(4)        3
*/
   d_dma_bps=d->pin_count>>3;
   bytes_per_slice=d->a_chan_cnt+d->d_tx_bps;
   #ifdef COPY_BUFFS
   for(int z=0;z<d->samples_per_half;z++){
      cbuf[z+cbufidx]=dbuf[z];
   }
   cbufidx+=d->samples_per_half;
   #endif
   samp_remain=d->samples_per_half;
   if((d->cont==false)&&((d->scnt+samp_remain)>(d->num_samples))){
        samp_remain=d->num_samples-d->scnt;
	//Dprintf("adj %d \n\r",samp_remain);
        d->scnt+=samp_remain;
   }else{
        d->scnt+=d->samples_per_half;
   }
   do{
     txbufidx=0;
     #ifdef TIME_STAMPS
     ts1=time_us_32();
     #endif
     samp_send=(bytes_per_tx/bytes_per_slice)>samp_remain ? samp_remain : (bytes_per_tx/bytes_per_slice);
     txbufidx=0;
     for(int s=0;s<samp_send;s++){
       lbyte=0;
       //Pull bytes from dma
       //With 17 or more channels we use 4 bytes of memory but only issue 3 bytes and thus d_dma_bps is > d_tx_bps, so 
       //make sure b stays less than both
       for(char b=0;((b<d_dma_bps)||d_dma_bps==0)&&(b<d->d_tx_bps);b++){
         cbyte=(dbuf[rxbufdidx]);
         rxbufdidx=(second_nibble |(d_dma_bps>0))?rxbufdidx+1: rxbufdidx;
         second_nibble^=1;
         txbuf[txbufidx]=(cbyte<<b)|lbyte|0x80;
         lbyte=cbyte>>(7-b);
         #ifdef SLICE_DBG
         Dprintf("TxD: b %d cbyte %X lbyte %X txbuf %X ridx %d tidx %d 2nd %d\n\r",b,cbyte,lbyte,txbuf[txbufidx],rxbufdidx,txbufidx,second_nibble);
         #endif
	 txbufidx++;
       }
       //If we transmit more bytes than we pull from DMA then send the extra byte without advancing read pointer
       //For instance in 8 channels we use 1 byte of DMA, but send 2 bytes on the wire 
       //Special case the 0 value of 4 channel
       if((d->d_tx_bps>d_dma_bps)&&(d_dma_bps>0)){
         txbuf[txbufidx]=lbyte|0x80;
         #ifdef SLICE_DBG
         Dprintf("TxDe:  lbyte %X txbuf %X ridx %d tidx %d\n\r",lbyte,txbuf[txbufidx],rxbufdidx,txbufidx);
         #endif
         txbufidx++;
       }
       //If we pull more bytes from DMA then we send on the wire then, increment the index around the extra bytes
       //For instance with 17 channels we use 8 nibbles (32 bits) of DMA memory, but only send 3 bytes
       if(d->d_tx_bps<d_dma_bps){
         #ifdef SLICE_DBG
         Dprintf("TxSkip:");
         #endif         
         rxbufdidx+=(d_dma_bps-d->d_tx_bps);
       }
       //Send the analog bytes
       for(char i=0;i<d->a_chan_cnt;i++){
         txbuf[txbufidx]=(abuf[rxbufaidx]>>1)|0x80;
         #ifdef SLICE_DBG
         Dprintf("TxA: i %d rx 0x%X txbuf 0x%X ridx %d tidx %d\n\r",i,abuf[rxbufaidx],txbuf[txbufidx],rxbufaidx,txbufidx);
         #endif
         txbufidx++;
         rxbufaidx++;
       }
     }//for s  
     #ifdef TIME_STAMPS
     ts2=time_us_32();
     #endif
     my_stdio_usb_out_chars(txbuf,txbufidx);
     #ifdef COPY_SER
     int z;
     for(z=0;(z<txbufidx)&&(z+serbufidx<SERBUF_SIZE);z++){
        serbuf[z+serbufidx]=txbuf[z];
     }
     serbufidx+=z;
     if(serbufidx>=SERBUF_SIZE){Dprintf("Serr3 ERROR %d\n\r",serbufidx);}
     #endif
     #ifdef TIME_STAMPS
     ts3=time_us_32();
     t1[tidx]=ts2-ts1;
     t2[tidx]=ts3-ts2;
     t3[tidx]=txbufidx;
     lenh[tidx]=samp_send;
     tidx=(tidx+1)%64;
     #endif
     samp_remain-=samp_send;
     ccnt+=txbufidx;
   }while(samp_remain);
   return ccnt;
}//send_slices

//See if a given half's dma engines are idle and if so process the data, update the write pointer and
//ensure that when done the other dma is still busy indicating we didn't lose data .
int __not_in_flash_func(check_half)(sr_device_t *d,volatile uint32_t *tstsa0,volatile uint32_t *tstsa1,volatile uint32_t *tstsd0,
                 volatile uint32_t *tstsd1,volatile uint32_t *t_addra0,volatile uint32_t *t_addrd0,
                 uint8_t *d_start_addr,uint8_t *a_start_addr, bool mask_xfer_err){
  int a0busy,d0busy;
  uint64_t stime,etime,dtime;
  volatile uint32_t *piodbg1,*piodbg2;
  volatile uint8_t piorxstall1,piorxstall2;
  stime=time_us_64();
  a0busy=((*tstsa0)>>24)&1;
  d0busy=((*tstsd0)>>24)&1;

  if(((a0busy==0)||(d->a_mask==0))
     &&((d0busy==0)||(d->d_mask==0))){
       //Use two dma controllers where each writes half of the trace space.
       //When one half finishes we send it's data while the other dma controller writes to the other half.
       //We rely on DMA chaining where one completing dma engine triggers the next.
       //When chaining happens the original target address is not restored so we must rewrite the starting address.
       //Also when chaining is enabled we can get into an infinite loop where we ping pong between each other.
       //and it is possible that if the other DMA controller finishes before we have send our half of the data that
       //our DMA controller will startup and overwrite our data.  That is the DMA overflow condition.
       //We don't actually overflow the DMA, but instead we fail to establish proper chaining to keep the
       //ping pong going.
       //The only way to avoid the overflow condition is to reduce the sampling rate so that the transmit of samples
       //can keep up, or do a fixed sample that fits into the sample buffer.  
       //When we first enter the loop it is assumed that the other controller is not chained to us so that if we don't
       //process in time then our DMA controller won't be started by the other.  That assumption is maintained by
       //always pointing our DMA engine to itself so tht in the next function call it is the "other" controller.
       //We then process our half's samples and send to USB.
       //When done we confirm that the other channel is still active, and if so we know we haven't overflowed and thus
       //can establish the chain from the other channel to us.  If the other channel is not still active, or if the 
       //PIO/ADC FIFOs indicates an RXstall condition which indicates PIO lost samples, then we abort.  
       //Note that in all cases we should never actually send any corrupted data we just send less than what was requested.
       //Note that we must use the "alias" versions of the DMA CSRs to prevent writes from triggering them.
       //Since we swap the csr pointers we determine the other half from the address offsets.
       uint8_t myachan=(((uint32_t) tstsa0)>>6)&0xF;
       uint8_t otherachan=(((uint32_t) tstsa1)>>6)&0xF;
       uint8_t mydchan=(((uint32_t)tstsd0)>>6)&0xF;
       uint8_t otherdchan=(((uint32_t)tstsd1)>>6)&0xF;
       //  Dprintf("my stts pre a 0x%X d 0x%X\n\r",*tstsa0,*tstsd0); 
       //Set my chain to myself so that I can't chain to the other. 
       volatile uint32_t ttmp;
       ttmp=((tstsd0[1])&0xFFFF87FF)|mydchan<<11;
       tstsd0[1]=ttmp;
       ttmp=((tstsa0[1])&0xFFFF87FF)|myachan<<11;
       tstsa0[1]=ttmp;
       (*t_addra0)=(uint32_t) a_start_addr;
       (*t_addrd0)=(uint32_t) d_start_addr;


/*     //This prints the raw DMA'd values if enabled
       if (d->a_mask) {
            Dprintf("Analog\n\r");
           for(int k=0;k<10;k++) Dprintf("%0X ",*(a_start_addr+k));
       }
       if (d->d_mask){
            Dprintf("\n\rDig\n\r");
            for(int k=0;k<200;k++) Dprintf("%0X",*(d_start_addr+k));
       }
       Dprintf("\n\r");
*/

       piodbg1=(volatile uint32_t *)(PIO0_BASE+0x8); //PIO DBG
       piorxstall1=(((*piodbg1)&0x1)&&(d->d_mask!=0));

       if((d->a_mask==0)&&((d->d_mask&0xFFFFFFF0)==0)){
          d->byte_cnt+=send_slices_D4(d,d_start_addr);
      }else{
	  d->byte_cnt+=send_slices(d,d_start_addr,a_start_addr);
      }

       if((d->cont==false)&&(d->scnt>=d->num_samples)){
         d->sending=false;
       }

       //Set my other chain to me
       //use aliases here as well to prevent triggers
       ttmp=((tstsd1[1])&0xFFFF87FF)|mydchan<<11;
       tstsd1[1]=ttmp;
       ttmp=((tstsa1[1])&0xFFFF87FF)|myachan<<11;
       tstsa1[1]=ttmp;
       num_halves++;
       piodbg2=(volatile uint32_t *)(PIO0_BASE+0x8); //PIO DBG
       piorxstall2=((*piodbg2)&0x1)&&(d->d_mask!=0);
       volatile uint32_t *adcfcs;
       uint8_t adcfail;
       adcfcs=(volatile uint32_t *)(ADC_BASE+0x8);//ADC FCS
       adcfail=(((*adcfcs) & 0xC00)&&(d->a_mask)) ? 1 : 0;
       //if(adcfail){Dprintf("adcfcs 0x%X %p\n\r",*adcfcs,(void *) adcfcs);}
       //Ensure other dma is still busy, if not that means we have samples from PIO/ADC that could be lost.
       //It's only an error if we haven't already gotten the samples we need, or if we are processing the first
       //half and all the remaining samples we need are in the 2nd half.
       //Note that in continuous mode num_samples isn't defined.
       uint8_t proc_fail;
       proc_fail=(~(((((*tstsa1)>>24)&1)||(d->a_mask==0))
		    &&((((*tstsd1)>>24)&1)||(d->d_mask==0)))&1);
       //Dprintf("pf 0x%X 0x%X %d\n\r",*tstsa1,*tstsd1,proc_fail);
       //       if(mask_xfer_err
       //     || ((piorxstall1==0)
       //      &&((((*tstsa1)>>24)&1)||(d->a_mask==0))
       //		  &&((((*tstsd1)>>24)&1)||(d->d_mask==0)))){
       if(mask_xfer_err
	      || ((piorxstall1==0)
                  &&(adcfail==0)
                  &&(piorxstall2==0)
                  &&(proc_fail==0))){
#ifdef TIME_STAMPS
           etime=time_us_64();
	   dtime=etime-stime;
           ttl[ttlidx]=dtime;
           ttlidx=(ttlidx+1)%64;
#endif
	   //           Dprintf("h\n\r");
           return 1;
        }else{
#ifdef TIME_STAMPS
           etime=time_us_64();
	   dtime=etime-stime;
           ttl[ttlidx]=dtime;
           ttlidx=(ttlidx+1)%64;
#endif

           if(piorxstall1 || piorxstall2){
	     Dprintf("***Abort PIO RXSTALL*** %d %d half %d\n\r",piorxstall1,piorxstall2,num_halves);
           }
           if(proc_fail){
               Dprintf("***Abort DMA ovrflow*** half %d \n\r",num_halves);
	   }
           if(adcfail){
               Dprintf("***Abort ADC ovrflow*** half %d \n\r",num_halves);
	   }
           d->aborted=true;
           //Issue end of trace markers to host
           //The main loop also sends these periodically until the host is done..
 	   my_stdio_usb_out_chars("!!!",3);//todo-cleanup        printf("!!!");
           //Dprintf("scnt %u \n\r",d->scnt);
           //Dprintf("a st %u msk %u\n\r",(*tstsa1),d->a_mask);
           //Dprintf("d st %u msk %u\n\r",(*tstsd1),d->d_mask);
           return -1;
        }
  }//if not busy
  return 0;
}//check_half

int __not_in_flash_func(main)(){
    char cmdstr[20];
    int cmdstrptr=0;
    char charin,tmpchar;
    long int i,j;
    uint16_t len;
    uint32_t tmpint,tmpint2;


    dma_channel_config acfg0,acfg1,pcfg0,pcfg1;
    uint admachan0,admachan1,pdmachan0,pdmachan1;
    PIO pio = pio0;
    uint piosm=0;
    float ddiv;
    int res;
    bool mask_xfer_err;
    bool init_done=false;
    uint64_t starttime,endtime;
    //125Mhz is the maximum official frequency.
    //Overclockers have gone to 300Mhz and beyond with simple testing, though they seem to test a very small
    //subset of the chip.
    //This code has been ran up to 240Mhz with no apparent issues, but for the most part
    //the biggest performance limiter is USB data rate which fast sysclks don't help.
    //And analog sampling is still limited to 500khz.
    //We set it to 120Mhz so that the PIO at maximum speed is an integer divisor.
    set_sys_clock_khz(120000,true);
    
    stdio_usb_init();
    uart_set_format(uart0,8,1,1);
    uart_init(uart0,115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);   
    sleep_us(100000);    
    Dprintf("\n\rHello from PICO sigrok device \n\r");


    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    Dprintf("pll_sys = %dkHz\n\r", f_pll_sys);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    Dprintf("clk_sys = %dkHz\n\r", f_clk_sys);

    //Set GPIO23 (TP4) to control switched mode power supply noise
    gpio_init_mask(1<<23);
    gpio_set_dir_masked(1<<23,1<<23);
    gpio_put_masked(1<<23,1<<23);
    //Early CDC IO code had lots of sleep statements, but the TUD code seems to have sufficient
    //checks that this isn't needed, but it doesn't hurt...
    sleep_us(100000);    
    int lowerhalf; //are we processing the upper or lower half of the data buffers
    //GPIOs 26 through 28 (the ADC ports) are on the PICO, GPIO29 is not a pin on the PICO
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_init();
    multicore_launch_core1(core1_code);


    admachan0 = dma_claim_unused_channel(true);
    admachan1 = dma_claim_unused_channel(true);
    pdmachan0 = dma_claim_unused_channel(true);
    pdmachan1 = dma_claim_unused_channel(true);
    acfg0 = dma_channel_get_default_config(admachan0);
    acfg1 = dma_channel_get_default_config(admachan1);
    pcfg0 = dma_channel_get_default_config(pdmachan0);
    pcfg1 = dma_channel_get_default_config(pdmachan1);
    //ADC transfer 8 bytes, PIO transfer the 4B default
    channel_config_set_transfer_data_size(&acfg0, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&acfg1, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&pcfg0, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&pcfg1, DMA_SIZE_32);
    channel_config_set_read_increment(&acfg0, false);
    channel_config_set_read_increment(&acfg1, false);
    channel_config_set_read_increment(&pcfg0, false);
    channel_config_set_read_increment(&pcfg1, false);
    channel_config_set_write_increment(&acfg0, true);
    channel_config_set_write_increment(&acfg1, true);
    channel_config_set_write_increment(&pcfg0, true);
    channel_config_set_write_increment(&pcfg1, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&acfg0, DREQ_ADC);
    channel_config_set_dreq(&acfg1, DREQ_ADC);
    volatile uint32_t *tcounta0,*tcounta1,*tcountd0,*tcountd1;
    tcounta0=(volatile uint32_t *)(DMA_BASE+0x40*admachan0+0x8); //DMA_TRANS_COUNT offset
    tcounta1=(volatile uint32_t *)(DMA_BASE+0x40*admachan1+0x8); //DMA_TRANS_COUNT offset
    tcountd0=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan0+0x8); //DMA_TRANS_COUNT offset
    tcountd1=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan1+0x8); //DMA_TRANS_COUNT offset

    //Use the debug version of the count registers because the count of the DMA_TRANS_COUNT is not
    //visible if it is not active
    volatile uint32_t *tcountdbga0,*tcountdbga1,*tcountdbgd0,*tcountdbgd1;
    tcountdbga0=(volatile uint32_t *)(DMA_BASE+0x40*admachan0+0x804); //DMA_TRANS_COUNT DBGoffset
    tcountdbga1=(volatile uint32_t *)(DMA_BASE+0x40*admachan1+0x804); //DMA_TRANS_COUNT DBGoffset
    tcountdbgd0=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan0+0x804); //DMA_TRANS_COUNT DBGoffset
    tcountdbgd1=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan1+0x804); //DMA_TRANS_COUNT DBGoffset
    volatile uint32_t *taddra0,*taddra1,*taddrd0,*taddrd1;
    taddra0=(volatile uint32_t *)(DMA_BASE+0x40*admachan0+0x4); //DMA_WRITE_addr offset
    taddra1=(volatile uint32_t *)(DMA_BASE+0x40*admachan1+0x4); //DMA_WRITE_addr offset
    taddrd0=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan0+0x4); //DMA_WRITE_addr offset
    taddrd1=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan1+0x4); //DMA_WRITE_addr offset
    volatile uint32_t *tstsa0,*tstsa1,*tstsd0,*tstsd1;
    tstsa0=(volatile uint32_t *)(DMA_BASE+0x40*admachan0+0xc); //DMA_WRITE_sts offset
    tstsa1=(volatile uint32_t *)(DMA_BASE+0x40*admachan1+0xc); //DMA_WRITE_sts offset
    tstsd0=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan0+0xc); //DMA_WRITE_sts offset
    tstsd1=(volatile uint32_t *)(DMA_BASE+0x40*pdmachan1+0xc); //DMA_WRITE_sts offset
    //PIO status
    volatile uint32_t *pioctrl,*piofstts,*piodbg,*pioflvl;
    pioctrl=(volatile uint32_t *)(PIO0_BASE); //PIO CTRL
    piofstts=(volatile uint32_t *)(PIO0_BASE+0x4); //PIO FSTAT
    piodbg=(volatile uint32_t *)(PIO0_BASE+0x8); //PIO DBG
    pioflvl=(volatile uint32_t *)(PIO0_BASE+0x10); //PIO FLVL

    volatile uint32_t *pio0sm0clkdiv;
    pio0sm0clkdiv=(volatile uint32_t *)(PIO0_BASE+0xc8); 
    //Give High priority to DMA to ensure we don't overflow the PIO or DMA fifos
    //The DMA controller must read across the common bus to read the PIO fifo so enabled both reads and write
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    
    init(&dev);
    //Since RP2040 is 32 bit this should always be 4B aligned, and it must be because the PIO
    //does DMA on a per byte basis
    //If either malloc fails the code will just hang
    Dprintf("Malloc\n\r");
    capture_buf=malloc(DMA_BUF_SIZE);
    Dprintf("DMA start %p\n\r",(void *)capture_buf);
    //Ensure we leave 10k or more left for any dynamic allocations that need it
    uint8_t *tptr;
    tptr=malloc(10000);
    Dprintf("10K free start %p\n\r",(void *)tptr);
    free(tptr); 

//This testmode forces the device into a capture state without needing
//sigrok cli/pulseview to initiate it
//All of the needed fields may not be up to date...
/*
   dev.started=false;
   dev.cont=false;
   dev.sending=true;
   dev.a_chan_cnt=0; //count of enabled analog channels
   dev.a_mask=0x0;
   dev.d_mask=0xF;
   dev.d_nps=1; //digital nibbles per slice
   dev.sample_rate=1000;
   dev.num_samples=5000;
   dev.scnt=0; //number of samples sent
   dev.byte_cnt=0;

*/

   gpio_init_mask(GPIO_D_MASK); //set as GPIO_FUNC_SIO and clear output enable
   gpio_set_dir_masked(GPIO_D_MASK,0);  //Set all to input
    while(1){
          __sev();//send event to wake core1
          if(send_resp){
            int mylen=strlen(dev.rspstr);
            //Don't mix printf with direct to usb commands
	    //printf("%s",dev.rspstr);
	    my_stdio_usb_out_chars(dev.rspstr,mylen);
            send_resp=false;
           }
         //Dprintf("ss %d %d",dev.sending,dev.started);
         if(dev.sending && (dev.started==false)) {
           lowerhalf=1;
#ifdef COPY_BUFFS
           cbufidx=0;
#endif
#ifdef COPY_SER
           serbufidx=0;
#endif
#ifdef TIME_STAMPS
           for (int x=0;x<64;x++){
            t1[x]=0; t2[x]=0;t3[x]=0;lenh[x]=0;ttl[x]=0;
           }
           tidx=0;
           ttlidx=0;
#endif
           //Sample rate must always be even.  Pulseview code enforces this 
           //because a frequency step of 2 is required to get a pulldown to specify
           //the sample rate, but sigrok cli can still pass it.
           dev.sample_rate>>=1;
           dev.sample_rate<<=1;
           uint8_t cfg_bits;
           //Bits 2:1 are special cfg bits
           //Bit 2 enables PIO level triggering
           //Bit 1 sets the level value for PIO level triggering
           cfg_bits=(dev.sample_rate%10)&0x6;
           dev.sample_rate-=cfg_bits;
           //Adjust up and align to 4 to avoid rounding errors etc
           if(dev.num_samples<16){dev.num_samples=16;}
           dev.num_samples=(dev.num_samples+3)&0xFFFFFFFC;
           //Divide capture buf evenly based on channel enables
           //d_size is aligned to 4 bytes because pio operates on words
           //These are the sizes for each half buffer in bytes
           //Calculate relative size in terms of nibbles which is the smallest unit, thus a_chan_cnt is multiplied by 2
           //For instance a D0..D3 with A0 would give 1/3rd the storage to digital and 2/3 to analog
           uint32_t d_nibbles,a_nibbles,t_nibbles; //digital, analog and total nibbles
           d_nibbles=dev.d_nps;  //digital is in grous of 4 bits
           a_nibbles=dev.a_chan_cnt*2; //1 byte per sample 
           t_nibbles=d_nibbles+a_nibbles;

//total buf size must be a multiple of a_nibbles*2, d_nibbles*8, and t_nibbles so that division is always 
//in whole samples
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
	   //	   Dprintf("Initial buf calcs nibbles d %d a %d t %d \n\r",d_nibbles,a_nibbles,t_nibbles);
           //Dprintf("chunk size %d samples %d buff %d needed %d\n\r",chunk_size,chunk_samples,buff_chunks,chunks_needed);
           //Dprintf("dbytes per chunk %d dig samples per chunk %d\n\r",dig_bytes_per_chunk,dig_samples_per_chunk);
           //If all of the samples we need fit in two half buffers or less then we can mask the error
           //logic that is looking for cases where we didn't send one half buffer to the host before
           //the 2nd buffer ended because we only use each half buffer once.
           mask_xfer_err=0;
           //If requested samples are smaller than the buffer, reduce the size so that the 
           //transfer completes sooner
           //Don't do this in continuous mode as the final size is unknown
           if(dev.cont==false){
               if(buff_chunks>chunks_needed){
                  mask_xfer_err=1;
                  buff_chunks=chunks_needed;
                  //Dprintf("Reduce buf chunks to %d\n\r",buff_chunks);
               }
           }
           //Give dig and analog equal fractions
           //This is the size of each half buffer in bytes
           dev.d_size=(buff_chunks*chunk_size*d_nibbles)/(t_nibbles*2);
           dev.a_size=(buff_chunks*chunk_size*a_nibbles)/(t_nibbles*2);
           dev.samples_per_half=chunk_samples*buff_chunks/2;
           //Dprintf("Final sizes d %d a %d mask err %d samples per half %d\n\r"
           //,dev.d_size,dev.a_size,mask_xfer_err,dev.samples_per_half);

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
          //Enable the initial chaing from the first half to 2nd, further chains are enabled based 
          //on whether we can parse each half in time.
          channel_config_set_chain_to(&acfg0,admachan1);
          channel_config_set_chain_to(&pcfg0,pdmachan1);	  


          num_halves=0;
          dev.dbuf0_start=0;
          dev.byte_cnt=0;
          dev.dbuf1_start=dev.d_size;
          dev.abuf0_start=dev.dbuf1_start+dev.d_size;
          dev.abuf1_start=dev.abuf0_start+dev.a_size;


          volatile uint32_t *adcdiv;
          adcdiv=(volatile uint32_t *)(ADC_BASE+0x10);//ADC DIV
          //   Dprintf("adcdiv start %u\n\r",*adcdiv);
	  //	  Dprintf("starting d_nps %u a_chan_cnt %u d_size %u a_size %u a_mask %X\n\r"
          //         ,dev.d_nps,dev.a_chan_cnt,dev.d_size,dev.a_size,dev.a_mask);
          //Dprintf("start offsets d0 0x%X d1 0x%X a0 0x%X a1 0x%X samperhalf %u\n\r"
          //    ,dev.dbuf0_start,dev.dbuf1_start,dev.abuf0_start,dev.abuf1_start,dev.samples_per_half);
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
             //-Values below 96 don't work well (the SDK has comments about it
             //in the adc_set_clkdiv document)
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
               *adcdiv=0;
             }else{
	       *adcdiv=((adcdivint-1)<<8)|adc_frac_int; 
             }
             //Dprintf("adcdiv %u frac %d\n\r",*adcdiv,adc_frac_int);

             //This is needed to clear the AINSEL so that when the round robin arbiter starts we start sampling on channel 0
             adc_select_input(0);
             adc_set_round_robin(dev.a_mask & 0x7);
             //             en, dreq_en,dreq_thresh,err_in_fifo,byte_shift to 8 bit
             adc_fifo_setup(true, true,   1,           false,       true);


             //set chan0 to immediate trigger (but without adc_run it shouldn't start), chan1 is chained to it.
             // channel, config, write_addr,read_addr,transfer_count,trigger)
             dma_channel_configure(admachan0,&acfg0,&(capture_buf[dev.abuf0_start]),&adc_hw->fifo,dev.a_size,true);
             dma_channel_configure(admachan1,&acfg1,&(capture_buf[dev.abuf1_start]),&adc_hw->fifo,dev.a_size,false);
             adc_fifo_drain();
          }//any analog enabled
          if(dev.d_mask){
             //analyzer_init from pico-examples
             //Due to how PIO shifts in bits, if any digital channel within a group of 8 is set, 
             //then all groups below it must also be set. We further restrict it in the tx_init function
            //by saying digital channel usage must be continous.
             dev.pin_count=0 ;
             if(dev.d_mask&0x0000000F) dev.pin_count+=4;
             if(dev.d_mask&0x000000F0) dev.pin_count+=4;
             if(dev.d_mask&0x0000FF00) dev.pin_count+=8;
             if(dev.d_mask&0x0FFF0000) dev.pin_count+=16;
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
             //start at GPIO2 (keep 0 and 1 for uart)
             sm_config_set_in_pins(&c, 2);
             sm_config_set_wrap(&c, offset, offset);

             uint16_t div_int;              
             uint8_t frac_int;
             div_int=frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000/dev.sample_rate;
             if(div_int<1) div_int=1;
             frac_int=(uint8_t)(((frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000%dev.sample_rate)*256ULL)/dev.sample_rate);
	     //             Dprintf("PIO sample clk %u divint %d divfrac %d \n\r",dev.sample_rate,div_int,frac_int);
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

#ifndef NODMA
	     channel_config_set_dreq(&pcfg0, pio_get_dreq(pio,piosm,false));
             channel_config_set_dreq(&pcfg1, pio_get_dreq(pio,piosm,false));
	     
	     //                       number    config   buffer target                  piosm          xfer size  trigger
             dma_channel_configure(pdmachan0,&pcfg0,&(capture_buf[dev.dbuf0_start]),&pio->rxf[piosm],dev.d_size>>2,true);
             dma_channel_configure(pdmachan1,&pcfg1,&(capture_buf[dev.dbuf1_start]),&pio->rxf[piosm],dev.d_size>>2,false);
#endif
             //Use bits from sample rate to enable hardware triggering.
             //Only supported on D2 and level detection only.
             //Only enabled if no analog channels are enabled because there is no way to
             //interlock the PIO starting and the ADC starting.
             //This just creates a wait state as the first instruction executed
             if(cfg_bits&4){
               if(dev.a_mask==0){
                 uint8_t level=((cfg_bits>>1)&1);
                 pio_sm_exec(pio, piosm, pio_encode_wait_gpio(level, 2));
                 Dprintf("DTRIG %d\n\r",level);
               }else{
                 Dprintf("ERR:No DTRIG with analog\n\r");
               }
             }
	     //This is done later so that we start everything as close in time as possible
	     //             pio_sm_set_enabled(pio, piosm, true);           
           } //dev.d_mask
          //These must be at their initial value,(or zero for the 2ndhalf) otherwise it indicates they have started to countdown
          //Dprintf("Tcount start d %u %u a %u %u\n\r",*tcountd0,*tcountd1,*tcounta0,*tcounta1);
          //These must be the initial value for both
          //Dprintf("Tcount dbg start d 0x%X 0x%X a 0x%X 0x%X\n\r",*tcountdbgd0,*tcountdbgd1,*tcountdbga0,*tcountdbga1);
         //These catch cases in DMA coding where DMA engines have started too soon..

          if((*tcountd0)!=(*tcountdbgd0)&&(dev.d_mask)){Dprintf("\n\r\n\rERROR: DMAD0 changing\n\r\n\r");}
          if((*tcounta0)!=(*tcountdbga0)&&(dev.a_mask)){Dprintf("\n\r\n\rERROR: DMAA0 changing\n\r\n\r");}
          if((*tcountd1)!=0){Dprintf("\n\r\n\rERROR: DMAD1 should start with 0 tcount\n\r\n\r");}
          if((*tcounta1)!=0){Dprintf("\n\r\n\rERROR: DMAA1 should start with 0 tcount\n\r\n\r");}

         // Dprintf("dma addr start d 0x%X 0x%X a 0x%X 0x%X\n\r",*taddrd0,*taddrd1,*taddra0,*taddra1);
          //Dprintf("capture_buf base %p \n\r",capture_buf);
          //Dprintf("capture_buf dig %p %p \n\r",&(capture_buf[dev.dbuf0_start]),&(capture_buf[dev.dbuf1_start]));
          //Dprintf("capture_buf analog %p %p\n\r",&(capture_buf[dev.abuf0_start]),&(capture_buf[dev.abuf1_start]));
          //Dprintf("PIOSMCLKDIV 0x%X\n\r",*pio0sm0clkdiv);

          //Dprintf("PIO ctrl 0x%X fstts 0x%X dbg 0x%X lvl 0x%X\n\r",*pioctrl,*piofstts,*piodbg,*pioflvl);
          //Dprintf("DMA channel assignments a %d %d d %d %d\n\r",admachan0,admachan1,pdmachan0,pdmachan1);
          //Dprintf("DMA ctr reg addrs a %p %p d %p %p\n\r",(void *) tstsa0,(void *)tstsa1,(void *)tstsd0,(void *)tstsd1);
          //Dprintf("DMA ctrl reg a 0x%X 0x%X d 0x%X 0x%X\n\r",*tstsa0,*tstsa1,*tstsd0,*tstsd1);
          //Enable logic and analog close together for best possible alignment
	  //warning - do not put printfs or similar things here...
	  tstart=time_us_32();
          adc_run(true); //enable free run sample mode
          pio_sm_set_enabled(pio, piosm, true);           
          dev.started=true;
          init_done=true;
        }//if dev.sending and not started
        if(dev.sending&& dev.started && ((dev.scnt<dev.num_samples)||dev.cont)){  
          uint32_t a,b;
          int ret;
          c0cnt++;
          if(lowerhalf){
 	     a=*piodbg;
             b=*pioflvl; 
             ret=check_half(&dev,tstsa0,tstsa1,tstsd0,tstsd1,taddra0,taddrd0,
               &(capture_buf[dev.dbuf0_start]),&(capture_buf[dev.abuf0_start]),mask_xfer_err);
             
             if(ret==1){
               lowerhalf=0;
	       //               Dprintf("Pd0 0x%X 0x%X %d\n\r",a,b,dev.byte_cnt);
             }
             else if(ret<0) {dev.sending=false;}
          }
          if(lowerhalf==0){
 	     a=*piodbg;
             b=*pioflvl; 
             ret=check_half(&dev,tstsa1,tstsa0,tstsd1,tstsd0,taddra1,taddrd1,
               &(capture_buf[dev.dbuf1_start]),&(capture_buf[dev.abuf1_start]),mask_xfer_err);
             if(ret==1) {
               lowerhalf=1;
               //Dprintf("Pd1 0x%X 0x%X %d\n\r",a,b,dev.byte_cnt);
             }
             else if(ret<0) {dev.sending=false;}
          }
         }//if sending and started and under numsamples

	//In high verbosity modes the host can miss the "!" so send these until it sends a "+"
	if(dev.aborted==true){
	  Dprintf("sending abort !\n\r");
	  my_stdio_usb_out_chars("!!!",3);//todo-cleanup        printf("!!!");
           sleep_us(200000);
        }
       //if we abort or normally finish a run sending gets dropped
       if((dev.sending==false)&&(init_done==true)){
	 //Dprintf("Ending PIO ctrl 0x%X fstts 0x%X dbg 0x%X lvl 0x%X\n\r",*pioctrl,*piofstts,*piodbg,*pioflvl);

             //Send the byte_cnt to ensure no bytes were lost
             if(dev.aborted==false){
               char brsp[16];
               //Give the host time to finish processing samples so that the bytecnt 
               //isn't dropped on the wire
               sleep_us(100000);
               Dprintf("Cleanup bytecnt %d\n\r",dev.byte_cnt);
  	       sprintf(brsp,"$%d%c",dev.byte_cnt,'+');
               puts_raw(brsp);
             }

#ifdef NODMA
	     // if dma is disabled and sample sizes are small this can be used
             // to pull the raw sample data from the pio fifos.
             uint lvl;
             lvl=pio_sm_get_rx_fifo_level(pio,piosm);
             Dprintf("FIFOlvl 0x%X\n\r",lvl);
             uint32_t fval;
             for (int x=0;x<lvl;x++){
               Dprintf("RX FIFO x %d:0x%X\n\r",x,pio_sm_get_blocking(pio,piosm));
             }

#endif
	     
	     


             adc_run(false);
             adc_fifo_drain();
             pio_sm_restart(pio, piosm);
             pio_sm_set_enabled(pio, piosm, false);
             pio_sm_clear_fifos(pio, piosm);
             pio_clear_instruction_memory(pio);

             dma_channel_abort(admachan0);
             dma_channel_abort(admachan1);
             dma_channel_abort(pdmachan0);
             dma_channel_abort(pdmachan1);
             init_done=false;

#ifdef COPY_BUFFS
             Dprintf("cbufidx %d\n\r",cbufidx);
             for(int z=0;z<cbufidx;z++){
                if(z%10==0){Dprintf(".%d ",z);}
                Dprintf("0x%03X ",cbuf[z]);
                if(z%10==9){Dprintf("\n\r");}
             }
             Dprintf("\n\r");
#endif
#ifdef COPY_SER
             Dprintf("serbufidx %d\n\r",serbufidx);
             for(int z=0;z<serbufidx;z++){
                if(z%10==0){Dprintf(".%d ",z);}
                Dprintf("0x%03X ",serbuf[z]);
                if(z%10==9){Dprintf("\n\r");}
             }
             Dprintf("\n\r");
#endif

#ifdef D4_COPY_BUFFS     
	     //Note: Cwcnt stops at 256 so only shows the first values
             Dprintf("Total cws: %d\n\r",d4cwcnt);
             for(int x=0;x<d4cwidx;x+=4){
	       Dprintf("x %d cw %X %X %X %X \n\r",x,d4cw[x],d4cw[x+1],d4cw[x+2],d4cw[x+3]);
               d4cw[x]=0;
             }
             d4cwidx=0;
             d4cwcnt=0;
#endif



           
#ifdef TIME_STAMPS
             uint32_t t1s=0,t2s=0,t3s=0,ls=0;
	     
             for (int x=0;x<32;x++){
              Dprintf("time idx %d t1 %u t2 %u t3 %u len %u\n\r",x,t1[x],t2[x],t3[x],lenh[x]);
              t1s+=t1[x];
              t2s+=t2[x];
              t3s+=t3[x];
              ls+=lenh[x];
             }
             Dprintf("Sums t1 %d t2 %d t3 %d l %d\n\r",t1s,t2s,t3s,ls);
             for (int x=0;x<20;x++){
	       //             Dprintf("ttl idx %d ttl %u\n\r",x,ttl[x]);
             }
#endif
             //Print out debug information after completing, rather than before so that it doesn't 
             //delay the start of a capture
             Dprintf("Complete: SRate %d NSmp %d\n\r",dev.sample_rate,dev.num_samples);
             Dprintf("Cont %d bcnt %d\n\r",dev.cont,dev.byte_cnt);
             Dprintf("DMsk 0x%X AMsk 0x%X\n\r",dev.d_mask,dev.a_mask);
             Dprintf("Half buffers %d sampperhalf %d\n\r",num_halves,dev.samples_per_half);
       

             //Report the number of main loops for each core to ensure
             //C0 runs more loops
             Dprintf("loop counts C0 %d C1 %d\n\r",c0cnt,c1cnt);
             c0cnt=0;
             c1cnt=0;
        }//i sending==false
   }//while(1)
 

}//main
