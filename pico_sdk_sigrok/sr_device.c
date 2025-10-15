#include "sr_device.h"
#include "hardware/uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int Dprintf(const char *fmt, ...)
{

  #if(UART_EN == 1)
   va_list argptr;
   int len = 1;
   char _dstr[256];

   memset(&_dstr, 0x00, sizeof(_dstr));
   va_start(argptr, fmt);
   len = vsprintf(_dstr, fmt, argptr);
   va_end(argptr);

   if ((len > 0) && (len < 240))
   {
      uart_puts(uart0, _dstr);
      uart_tx_wait_blocking(uart0);
   }
   else
   {
      uart_puts(uart0, "UART OVRFLW");
      uart_tx_wait_blocking(uart0);
   }
   return len;
  #else
   return 0;
   #endif
}

// reset as part of init, or on a completed send
void reset(sr_device_t *d)
{
   d->state==IDLE;
   d->cont = 0;
   d->scnt = 0;
   // d->notfirst=false;
   // Set triggered by default so that we don't check for HW triggers
   // unless the driver sends a trigger command
   // d->triggered=true;
   // d->tlval=0;
   // d->lvl0mask=0;
   // d->lvl1mask=0;
   // d->risemask=0;
   // d->fallmask=0;
   // d->chgmask=0;
};
// initial post reset state
void init(sr_device_t *d)
{
   reset(d);
   d->a_mask = 0;
   d->d_mask = 0;
   d->sample_rate = 5000;
   d->num_samples = 10;
   d->a_chan_cnt = 0;
   d->d_nps = 0;
   d->cmdstrptr = 0;
}
void tx_init(sr_device_t *d)
{
   // A reset should have already been called to restart the device.
   // An additional one here would clear trigger and other state that had been updated
   //     reset(d);
   d->a_chan_cnt = 0;
   for (int i = 0; i < NUM_A_CHAN; i++)
   {
      if (((d->a_mask) >> i) & 1)
      {
         d->a_chan_cnt++;
      }
   }
   // Nibbles per slice controls how PIO digital data is stored
   // Only support 0,1,2,4 or 8, which use 0,4,8,16 or 32 bits of PIO fifo data
   // per sample clock.
   d->d_nps = (d->d_mask & 0xF) ? 1 : 0;
   d->d_nps = (d->d_mask & 0xF0) ? (d->d_nps) + 1 : d->d_nps;
   d->d_nps = (d->d_mask & 0xFF00) ? (d->d_nps) + 2 : d->d_nps;
   d->d_nps = (d->d_mask & 0xFFFF0000) ? (d->d_nps) + 4 : d->d_nps;
   // Dealing with samples on a per nibble, rather than per byte basis in non D4 mode
   // creates a bunch of annoying special cases, so forcing non D4 mode to always store a minimum
   // of 8 bits.
   if ((d->d_nps == 1) && (d->a_chan_cnt > 0))
   {
      d->d_nps = 2;
   }

   // Digital channels must enable from bottom and go up, but that is checked by the host
   d->d_chan_cnt = 0;
   for (int i = 0; i < NUM_D_CHAN; i++)
   {
      if (((d->d_mask) >> i) & 1)
      {
         //    Dprintf("i %d inv %d mask %X\n\r",i,invld,d->d_mask);
         d->d_chan_cnt++;
      }
   }
   d->d_tx_bps = (d->d_chan_cnt + 6) / 7;
   d->state=STARTED;
}
// Process incoming character stream
// Return 1 if the device rspstr has a response to send to host
// Be sure that rspstr does not have \n  or \r.
int process_char(sr_device_t *d, char charin)
{
   int tmpint, tmpint2, ret;
   // set default rspstr for all commands that have a dataless ack
   d->rspstr[0] = '*';
   d->rspstr[1] = 0;
   // the reset character works by itself
   if (charin == '*')
   {
      reset(d);
      Dprintf("RST* %d\n\r", d->state);
      return 0;
   }
   else if ((charin == '\r') || (charin == '\n'))
   {
      d->cmdstr[d->cmdstrptr] = 0;
      switch (d->cmdstr[0])
      {
      //Enter boot_sel_mode. Require a long string to prevent accidental reboots
      case 'b':
         if(!strcmp(d->cmdstr,"bootsel")){
            Dprintf("Entering Bootsel mode\n\r");
            sleep_ms(1000);
            //reset with inputs per cold boot
            rom_reset_usb_boot(0,0);
         }else{
            Dprintf("Invald bootsel command - enter \"bootsel\"\n\r");
         } 
         ret=0;
         break;
      case 'i':
         // SREGEN,AxxyDzz,00 - num analog, analog size, num digital,version
         sprintf(d->rspstr, "SRPICO,A%02d1D%02d,02", NUM_A_CHAN, NUM_D_CHAN);
         Dprintf("ID rsp %s\n\r", d->rspstr);
         ret = 1;
         break;
      case 'R':
         tmpint = atol(&(d->cmdstr[1]));
         if ((tmpint >= 5000) && (tmpint <= 120000016))
         { // Add 16 to support cfg_bits
            d->sample_rate = tmpint;
            // Dprintf("SMPRATE= %u\n\r",d->sample_rate);
            ret = 1;
         }
         else
         {
            Dprintf("unsupported smp rate %s\n\r", d->cmdstr);
            ret = 0;
         }
         break;
      // sample limit
      case 'L':
         tmpint = atol(&(d->cmdstr[1]));
         if (tmpint > 0)
         {
            d->num_samples = tmpint;
            // Dprintf("NUMSMP=%u\n\r",d->num_samples);
            ret = 1;
         }
         else
         {
            Dprintf("bad num samples %s\n\r", d->cmdstr);
            ret = 0;
         }
         break;
      //get analog scale
      case 'a':
         tmpint = atoi(&(d->cmdstr[1])); // extract channel number
         if (tmpint >= 0)
         {
            // scale and offset are both in integer uVolts
            // separated by x
            sprintf(d->rspstr, "25700x0"); // 3.3/(2^7) and 0V offset
            // Dprintf("ASCL%d\n\r",tmpint);
            ret = 1;
         }
         else
         {
            Dprintf("bad ascale %s\n\r", d->cmdstr);
            ret = 1; // this will return a '*' causing the host to fail
         }
         break;
      case 'F': // fixed set of samples
         Dprintf("STRT_FIX\n\r");
         tx_init(d);
         d->cont = 0;
         ret = 0;
         break;
      case 'C': // continous mode
         tx_init(d);
         d->cont = 1;
         Dprintf("STRT_CONT\n\r");
         ret = 0;
         break;
      case 't': // trigger -format tvxx where v is value and xx is two digit channel
         /*HW trigger depracated
             tmpint=d->cmdstr[1]-'0';
                  tmpint2=atoi(&(d->cmdstr[2])); //extract channel number which starts at D2
             //Dprintf("Trigger input %d val %d\n\r",tmpint2,tmpint);
                  if((tmpint2>=2)&&(tmpint>=0)&&(tmpint<=4)){
                    d->triggered=false;
                    switch(tmpint){
                 case 0: d->lvl0mask|=1<<(tmpint2-2);break;
                 case 1: d->lvl1mask|=1<<(tmpint2-2);break;
                 case 2: d->risemask|=1<<(tmpint2-2);break;
                 case 3: d->fallmask|=1<<(tmpint2-2);break;
                 default: d->chgmask|=1<<(tmpint2-2);break;
               }
                    //Dprintf("Trigger channel %d val %d 0x%X\n\r",tmpint2,tmpint,d->lvl0mask);
               //Dprintf("LVL0mask 0x%X\n\r",d->lvl0mask);
                    //Dprintf("LVL1mask 0x%X\n\r",d->lvl1mask);
                    //Dprintf("risemask 0x%X\n\r",d->risemask);
                    //Dprintf("fallmask 0x%X\n\r",d->fallmask);
                    //Dprintf("edgemask 0x%X\n\r",d->chgmask);
                  }else{
               Dprintf("bad trigger channel %d val %d\n\r",tmpint2,tmpint);
                    d->triggered=true;
                  }
         */
         ret = 1;
         break;
      case 'p': // pretrigger count
         tmpint = atoi(&(d->cmdstr[1]));
         Dprintf("Pre-trigger samples %d cmd %s\n\r", tmpint, d->cmdstr);
         ret = 1;
         break;
      //Enable/disable Analog channel   
      // format is Axyy where x is 0 for disabled, 1 for enabled and yy is channel #
      case 'A':                          
         tmpint = d->cmdstr[1] - '0';     // extract enable value
         tmpint2 = atoi(&(d->cmdstr[2])); // extract channel number
         if ((tmpint >= 0) && (tmpint <= 1) && (tmpint2 >= 0) && (tmpint2 <= 31))
         {
            d->a_mask = d->a_mask & ~(1 << tmpint2);
            d->a_mask = d->a_mask | (tmpint << tmpint2);
            // Dprintf("A%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->a_mask);
            ret = 1;
         }
         else
         {
            ret = 0;
         }
         break;
      //Enable/disable digital channel.
      // format is Dxyy where x is 0 for disabled, 1 for enabled and yy is channel #
      //Note that this is a fixed number 0..N regardless of channel naming and/or pins enabled
      case 'D':                           /// enable digital channel always a set
         tmpint = d->cmdstr[1] - '0';     // extract enable value
         tmpint2 = atoi(&(d->cmdstr[2])); // extract channel number
         if ((tmpint >= 0) && (tmpint <= 1) && (tmpint2 >= 0) && (tmpint2 <= 31))
         {
            d->d_mask = d->d_mask & ~(1 << tmpint2);
            d->d_mask = d->d_mask | (tmpint << tmpint2);
            Dprintf("D%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->d_mask);
            ret = 1;
         }
         else
         {
            ret = 0;
         }
         break;
      //Get signal name.  The name can only be pulled from the device, there is no
      //set because the device has no use for the names.  Between the host and device
      //the names are always 0..N for all types, but this mode allows the PIN/board/device
      //names to be communicated.
      //format is n(A/D)xx where A/D selects analog digital, and x is one or two characters representing an integer
      //Note: this functional call may return names for channels that don't exist.  It's up
      //to the caller to only ask for names for channels reported in the identify
      case 'n':
          tmpint=atoi(&(d->cmdstr[2]));
          //Dprintf("Name %c %d \n\r", d->cmdstr[1],tmpint);
          if(d->cmdstr[1]=='D'){
//On standard PICO and PICO2
//GP0-22 are continous, and then GP26,27,28
//So in basemode, D0-D20 are GP2..GP22
//in dig_26_mode D0-D22,D23-D25 are GP0..GP22,GP26..GP28
//in dig_32_mode D0-D31 are GP0..GP31
            #ifdef BASE_MODE //D0-20 are GP2..GP22
               tmpint2=tmpint+2;
            #elif DIG_26_MODE //D0-D22,D23-D25 are GP0..GP22,GP26..GP28
               tmpint2=(tmpint<=22) ? tmpint : tmpint+3; 
            #else //DIG_32_MODE and all else are direct mapped.
               tmpint2=tmpint;
            #endif
            Dprintf("NameD %c %d %d\n\r", d->cmdstr[1],tmpint,tmpint2);
            sprintf(d->rspstr, "GP%d",tmpint2);
            ret=1;            
          } else  if(d->cmdstr[1]=='A'){
            //ADC0/1/2 are GP26,27,28
            tmpint2=tmpint+26;
            Dprintf("NameA %c %d %d\n\r", d->cmdstr[1],tmpint,tmpint2);
            sprintf(d->rspstr, "ADC%d_GP%d",tmpint,tmpint2);
            ret=1;            
          } else{
            ret=0;
          }
          break;
      default:
         Dprintf("bad command %s\n\r", d->cmdstr);
         ret = 0;
      } // case
      //        Dprintf("CmdDone %s\n\r",d->cmdstr);
      d->cmdstrptr = 0;
   }
   else // no CR/LF
   { if (d->cmdstrptr >= 19)
      {
         d->cmdstr[18] = 0;
         Dprintf("Command overflow %s\n\r", d->cmdstr);
         d->cmdstrptr = 0;
      }
      d->cmdstr[d->cmdstrptr++] = charin;
      ret = 0;
   } // else no CR/LF
   // default return 0 means to not send any kind of response
   return ret;
} // process_char
