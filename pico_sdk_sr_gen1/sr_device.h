//number of analog channels
//#define NUM_A_CHAN 3
//TODO - testing a D4 only build to see if it's faster because I don't
//need to replicate unused channel data
#define NUM_A_CHAN 0
//number of digital channels
//GP0 and 1 are reserved for uart
//GP23-25 are not on the board and 26-28 are ADC.
//Thus a max of 21 digital channels
//#define NUM_D_CHAN 21

#define NUM_D_CHAN 4
//Mask of bits 22:2 to use as inputs - 
#define GPIO_D_MASK 0x7FFFFC
//number of bytes per analog sample, must be 1,2,3,or 4
//TODO - make it just be always 1
#define A_BYTES 1
//enable RLE encoding
#define RLE_EN 1
//TODO -finalize a value
#define RLE_THRESH 2000
//number of chars between makers
#define STRIDE 128
//TODO - implement various limits on sample rate
#define MAX_SAMPLE_RATE 120000000
//warning - with a 256b limit in the Dprintf, large strings won't dprintf well
#define MRK_STR_SIZE 240 //TODO - larger for testing puts_raw_len
//TODO - long term prints don't belong in this code
int Dprintf(const char *fmt, ...)
{
    va_list argptr;
    int len = 1;
    char    _dstr[256];
     

     memset(&_dstr, 0x00, sizeof(_dstr) );
    va_start(argptr, fmt);
    len = vsprintf(_dstr, fmt, argptr);
    va_end(argptr);

        if ( (len > 0) && (len < 240 ) )
        {

    uart_puts(uart0,_dstr);
    uart_tx_wait_blocking(uart0);
//don't always want \n
//    uart_puts(uart0,"\r\n");
//    uart_tx_wait_blocking(uart0);
        }
         else{
         uart_puts(uart0,"UART OVRFLW");
        uart_tx_wait_blocking(uart0);
         
             }
    return len;
} 
typedef struct sr_device {
  uint32_t sample_rate;
  uint32_t num_samples;
  uint32_t a_mask,d_mask;
  uint32_t byte_cnt; //total serial bytes sent
  uint32_t samples_per_half; //number of samples for one of the 4 dma target arrays
  uint8_t a_chan_cnt; //count of enabled analog channels
  uint8_t d_chan_cnt; //count of enabled digital channels
  //Pins sampled by the PIO - 4,8,16 or 32
  uint8_t pin_count; 
  uint8_t d_nps; //digital nibbles per slice from a PIO/DMA perspective.
  uint32_t scnt; //number of samples sent
  char cmdstrptr;
  char cmdstr[20];//used for parsing input
  uint32_t d_size,a_size; //size of each of the two data buffers for each of a& d
  uint32_t dbuf0_start,dbuf1_start,abuf0_start,abuf1_start;
  //TODO optimize this - this is the size of string we send , for now deriving from sample size
  uint32_t send_size;
  char rspstr[20];
  //The current slice being created
  char slicestr[64];
  char slicestridx;
#ifdef RLE_EN
  char prevslicestr[64];
  uint32_t rle_cnt;
#endif
  //String with markers inserted used to send the final transmit data
  uint32_t mrkbyte_cnt;
  char mrkstr[MRK_STR_SIZE];
  uint16_t mrkstridx;
//mark key control variables voltatile since multiple cores access them
  volatile bool started;
  volatile bool sending;
  volatile bool cont;
  volatile bool aborted;
  } sr_device_t;

//Add one 4 bit equivalent character
//TODO - I don't think the inlines help.
void inline add_slice_char(sr_device_t *d,char bytein){
   d->slicestr[d->slicestridx++]=bytein;
};


//returns 0 if no string to send
//otherwise returns the length of the string that is in d->mrkstridx which needs to be printed
uint32_t slice_done(sr_device_t *d){
  bool almost_done;
//   int almost_done;
  almost_done =((d->cont==true)||(((d->num_samples)-(d->scnt))>2)) ? false: true;
//  almost_done =((((d->num_samples)-(d->scnt))>2)) ? false: true;
//  almost_done =((((d->num_samples)-(d->scnt))>2)) ? 0: 1;
  if(d->scnt>99900){
  //  Dprintf("sd cnt %u ad %d cont %d ns %d\n\r",d->scnt,almost_done,d->cont,d->num_samples);
  }
  //Dprintf("sd %d\n\r",d->slicestridx);
  //terminate the current slice string
  d->slicestr[d->slicestridx]=0;
  bool match;
#ifdef RLE_EN
  match=((d->scnt>0)&&(RLE_EN)&&(strncmp(d->prevslicestr,d->slicestr,d->slicestridx)==0)) ? true : false;
  strncpy(d->prevslicestr,d->slicestr,d->slicestridx);
//Only increment rle if we matched, as if we don't match we just flush the previous ones.
  if(match) d->rle_cnt++;  
  //If not a match we must drain all RLEs before sending the new sample.
  //The third condition ensures we don't have any rle counts that haven't been pushed to the marker 
  //string, so send all slices as we get close to the end.
  if(((match==false)&&(d->rle_cnt>0))
     ||(d->rle_cnt>=RLE_THRESH)
     ||((d->rle_cnt>0)&&(almost_done==true))){
       uint8_t rlenum;
       char rlechar;
//       Dprintf("push rle %d match %d slicestridx %d ad %d scnt %u\n\r",d->rle_cnt,match,d->slicestridx,almost_done,d->scnt);
       while(d->rle_cnt>256){
          rlenum=(d->rle_cnt>=4096) ? 16 : (d->rle_cnt>>8);
          //subtract the 1 because 256 is starting value, not 0
          rlechar=rlenum+96-1;
          d->rle_cnt-=rlenum<<8;
          insert_mark(d,rlechar);
          //Dprintf("rle256 %d char %d %c cnt %d",rlenum,rlechar,rlechar,rle_cnt);
        }
       while(d->rle_cnt>16){
          rlenum=(d->rle_cnt>=256) ? 16: (d->rle_cnt>>4);
          rlechar=rlenum+80-1;
          d->rle_cnt-=rlenum<<4;
          insert_mark(d,rlechar);
          //Dprintf("rle16 %d char %d %c cnt %d",rlenum,rlechar,rlechar,rle_cnt);
       }
       while(d->rle_cnt>0){
          rlenum=(d->rle_cnt>=16) ? 16: (d->rle_cnt);
          rlechar=rlenum+64-1;
          d->rle_cnt-=rlenum;
          insert_mark(d,rlechar);
          //Dprintf("rle1 %d char %d %c cnt %d",rlenum,rlechar,rlechar,rle_cnt);
        }
  }//(match==false) || rle_cnt
#else
  match=false;
#endif
  if(match==false){
//      Dprintf("nomatch pushes slice idx %d",slicestridx);
      for(int cnt=0;cnt<d->slicestridx;cnt++){
         insert_mark(d,d->slicestr[cnt]);
      }

  }
  //Empiricaly found that 16 bytes provides the highest bandwidth 
  //likely that's because we balance new sample generation with the ability to drain a packet.
  //TODO - validate that is still the case...
  uint32_t ret;  
  if(almost_done==true){
  //   Dprintf("ad %u\n\r",d->scnt);
  }
//  Dprintf("curr #%s# \n\r",d->mrkstr);
//  Dprintf("curr idx %d\n\r",d->mrkstridx);

//TODO - 16 was before I added the sleep.. trying larger payloads with smaller sleep
  if((d->mrkstridx>d->send_size)  //SPOT
    ||(d->mrkstridx>=(MRK_STR_SIZE-2))
    ||(almost_done==true)){
       d->mrkstr[d->mrkstridx]=0;
//       Dprintf("tx %s %d\n\r",d->mrkstr,d->mrkstridx);
       ret=d->mrkstridx;
//TODO - hack - for the len function rely on user to clear
//       d->mrkstridx=0;
     }else{
      ret=0;
  }
  d->scnt++;
  d->slicestridx=0;
  return ret;

};
//reset as part of init, or on a completed send
void reset(sr_device_t *d){
    d->sending=0;
    d->cont=0;
    d->aborted=false;
    d->started=false;
    d->mrkbyte_cnt=0;
    d->mrkstridx=0;
    d->slicestridx=0;
    d->scnt=0;
#ifdef RLE_EN
    d->rle_cnt=0;
#endif    
};  
//initial post reset state
void init(sr_device_t *d){
    reset(d);
    d->a_mask=0;
    d->d_mask=0;
    d->sample_rate=1000;
    d->num_samples=10;
    d->a_chan_cnt=0;
    d->d_nps=0;
    d->cmdstrptr=0;
}
void tx_init(sr_device_t *d){
    reset(d);
    d->a_chan_cnt=0;
    for(int i=0;i<NUM_A_CHAN;i++){
       if(((d->a_mask)>>i)&1){d->a_chan_cnt++;}
    }
    //Nibbles per slice controls how PIO digital data is stored
    //Only support 0,1,2,4 or 8, which use 0,4,8,16 or 32 bits of PIO fifo data
    //per sample clock.
    d->d_nps=(d->d_mask&       0xF) ? 1 : 0;
    d->d_nps=(d->d_mask&      0xF0) ? (d->d_nps)+1 : d->d_nps;
    d->d_nps=(d->d_mask&    0xFF00) ? (d->d_nps)+2 : d->d_nps;
    d->d_nps=(d->d_mask&0xFFFF0000) ? (d->d_nps)+4 : d->d_nps;
    
    if((d->d_nps==0)  &&(d-> a_chan_cnt==0)){
       Dprintf("Error no channels enabled\n\r");
       printf("!!!!!"); //stop transfer to host
       return;
    }
    //Digital channels must enable from D0 and go up
    int invld=0; //have we seen a digital channel that is not valid
    d->d_chan_cnt=0;
    for(int i=0;i<NUM_D_CHAN;i++){
       if(((d->d_mask)>>i)&1){
          Dprintf("i %d inv %d mask %X\n\r",i,invld,d->d_mask);
          if(invld){
            Dprintf("Error:Dig chan D%d (pin D%d) not continuous from 0 (pin D2)\n\r",i,i+2);
            printf("!!!!!"); //stop transfer to host
            return;
            
          }else{
            d->d_chan_cnt++;
          }
        }else{
          invld=1;
        }
    }


    d->sending=true;
    
}
//Since the sending of data on the link is device
//specific, return a non zero value to tell 
//the user code to issue a write of d->respstr
int process_char(sr_device_t *d,char charin){
   int tmpint,tmpint2,ret;
   //set default rspstr for all commands that have a dataless ack
   d->rspstr[0]='*';
   d->rspstr[1]=0;
   //the reset character works by itself
  if(charin=='*'){
     reset(d);
     Dprintf("RST* sending %d\n\r",d->sending);
     return 0;
  }else if((charin=='\r')||(charin=='\n')){
    d->cmdstr[d->cmdstrptr]=0;
    switch(d->cmdstr[0]){
    case 'i':
       //SREGEN,AxxyDzz,00 - num analog, analog size, num digital,version
       sprintf(d->rspstr,"SRGEN,A%02d1D%02d,00\n",NUM_A_CHAN,NUM_D_CHAN);
       Dprintf("ID rsp %s\n\r",d->rspstr);
       ret=1;
       break;
     case 'R':
       tmpint=atol(&(d->cmdstr[1]));
       if((tmpint>0)&&(tmpint<=MAX_SAMPLE_RATE)){
          d->sample_rate=tmpint;
          Dprintf("SMPRATE= %u\n\r",d->sample_rate);
          ret=1;
       }
       else {
          Dprintf("unsupported smp rate %s\n\r",d->cmdstr);
          ret=0;
       }
       break;
     //sample limit
     case 'L':
       tmpint=atol(&(d->cmdstr[1]));
       if(tmpint>0){
          d->num_samples=tmpint; 
           Dprintf("NUMSMP=%u",d->num_samples);
          ret=1;
       }else{
          Dprintf("unsupported num samples %s\n\r",d->cmdstr);
          ret=0;
       }
       break;
     case 'a':
         tmpint=atoi(&(d->cmdstr[1])); //extract channel number
         if(tmpint>=0){
            sprintf(d->rspstr,"0.0257,0.0\n");  //3.3/(2^7) and 0V offset
            Dprintf("ASCL%d\n\r",tmpint);
            ret=1;
         }else{
            Dprintf("bad ascale %s\n\r",d->cmdstr);
            ret=1; //this will return a '*' causing the host to fail
          }
           break;
      case 'F':  //fixed set of samples
           Dprintf("starting fixed dump\n\r");
           tx_init(d);
           d->cont=0;
           ret=0;
           break;
      case 'C':  //continous mode
            tx_init(d);
            d->cont=1;
            Dprintf("starting continous stream\n\r");
            ret=0;
            break;
      //format is Axyy where x is 0 for disabled, 1 for enabled and yy is channel #
      case 'A':  ///enable analog channel always a set
            tmpint=d->cmdstr[1]-'0'; //extract enable value
            tmpint2=atoi(&(d->cmdstr[2])); //extract channel number
            if((tmpint>=0)&&(tmpint<=1)&&(tmpint2>=0)&&(tmpint2<=31)){
               d->a_mask=d->a_mask & ~(1<<tmpint2);
               d->a_mask=d->a_mask | (tmpint<<tmpint2);
               Dprintf("A%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->a_mask);
               ret=1;
            }else{
               ret=0;
            }
            break;
      //format is Dxyy where x is 0 for disabled, 1 for enabled and yy is channel #
       case 'D':  ///enable digital channel always a set
            tmpint=d->cmdstr[1]-'0'; //extract enable value
            tmpint2=atoi(&(d->cmdstr[2])); //extract channel number
            if((tmpint>=0)&&(tmpint<=1)&&(tmpint2>=0)&&(tmpint2<=31)){
               d->d_mask=d->d_mask & ~(1<<tmpint2);
               d->d_mask=d->d_mask | (tmpint<<tmpint2);
               Dprintf("D%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->d_mask);
               ret=1;
             }else{
                ret=0;
             }
             break;
        default:
             Dprintf("bad command %s\n\r",d->cmdstr);
             ret=0;
        }//case
//        Dprintf("CmdDone %s\n\r",d->cmdstr);
        d->cmdstrptr=0;
      }else{ //no CR/LF
         if(d->cmdstrptr>=19){
            d->cmdstr[18]=0;
            Dprintf("Command overflow %s\n\r",d->cmdstr);
            d->cmdstrptr=0;
          }
          d->cmdstr[d->cmdstrptr++]=charin;
          ret=0;
       }//else
//default return 0 means to not send any kind of response
   return ret;
}//process_char


