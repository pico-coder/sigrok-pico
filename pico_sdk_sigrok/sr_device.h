//Pin usage
//GP0 and 1 are reserved for debug uart
//GP2-GP22 are digital inputs
//GP23 controls power supply modes and is not a board input
//GP24-25 are not on the board and not used
//GP26-28 are ADC.
//number of analog channels
#define NUM_A_CHAN 3
//number of digital channels
#define NUM_D_CHAN 21
//Mask of bits 22:2 to use as inputs - 
#define GPIO_D_MASK 0x7FFFFC


int __not_in_flash_func(Dprintf)(const char *fmt, ...)
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
  uint8_t d_tx_bps;   //Digital Transmit bytes per slice
  //Pins sampled by the PIO - 4,8,16 or 32
  uint8_t pin_count; 
  uint8_t d_nps; //digital nibbles per slice from a PIO/DMA perspective.
  uint32_t scnt; //number of samples sent
  char cmdstrptr;
  char cmdstr[20];//used for parsing input
  uint32_t d_size,a_size; //size of each of the two data buffers for each of a& d
  uint32_t dbuf0_start,dbuf1_start,abuf0_start,abuf1_start; //starting memory pointers of adc buffers
  char rspstr[20];
//mark key control variables voltatile since multiple cores might access them
  volatile bool started;
  volatile bool sending;
  volatile bool cont;
  volatile bool aborted;
  } sr_device_t;


//reset as part of init, or on a completed send
void __not_in_flash_func(reset)(sr_device_t *d){
    d->sending=0;
    d->cont=0;
    d->aborted=false;
    d->started=false;
    d->scnt=0;

};  
//initial post reset state
void __not_in_flash_func(init)(sr_device_t *d){
    reset(d);
    d->a_mask=0;
    d->d_mask=0;
    d->sample_rate=5000;
    d->num_samples=10;
    d->a_chan_cnt=0;
    d->d_nps=0;
    d->cmdstrptr=0;
}
void __not_in_flash_func(tx_init)(sr_device_t *d){
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
    
    //Digital channels must enable from D0 and go up, but that is checked by the host
    d->d_chan_cnt=0;
    for(int i=0;i<NUM_D_CHAN;i++){
       if(((d->d_mask)>>i)&1){
	 //    Dprintf("i %d inv %d mask %X\n\r",i,invld,d->d_mask);
          d->d_chan_cnt++;
       }
    }
    d->d_tx_bps=(d->d_chan_cnt+6)/7;
    d->sending=true;
    
}
//Process incoming character stream
//Return 1 if the device rspstr has a response to send to host
//Be sure that rspstr does not have \n  or \r.
int __not_in_flash_func(process_char)(sr_device_t *d,char charin){
   int tmpint,tmpint2,ret;
   //set default rspstr for all commands that have a dataless ack
   d->rspstr[0]='*';
   d->rspstr[1]=0;
   //the reset character works by itself
  if(charin=='*'){
     reset(d);
     Dprintf("RST* %d\n\r",d->sending);
     return 0;
  }else if((charin=='\r')||(charin=='\n')){
    d->cmdstr[d->cmdstrptr]=0;
    switch(d->cmdstr[0]){
    case 'i':
       //SREGEN,AxxyDzz,00 - num analog, analog size, num digital,version
       sprintf(d->rspstr,"SRPICO,A%02d1D%02d,00",NUM_A_CHAN,NUM_D_CHAN);
       Dprintf("ID rsp %s\n\r",d->rspstr);
       ret=1;
       break;
     case 'R':
       tmpint=atol(&(d->cmdstr[1]));
       if((tmpint>=5000)&&(tmpint<=120000016)){ //Add 16 to support cfg_bits
          d->sample_rate=tmpint;
          //Dprintf("SMPRATE= %u\n\r",d->sample_rate);
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
          //Dprintf("NUMSMP=%u\n\r",d->num_samples);
          ret=1;
       }else{
          Dprintf("bad num samples %s\n\r",d->cmdstr);
          ret=0;
       }
       break;
     case 'a':
         tmpint=atoi(&(d->cmdstr[1])); //extract channel number
         if(tmpint>=0){
	   //scale and offset are both in integer uVolts
           //separated by x
            sprintf(d->rspstr,"25700x0");  //3.3/(2^7) and 0V offset
            //Dprintf("ASCL%d\n\r",tmpint);
            ret=1;
         }else{
            Dprintf("bad ascale %s\n\r",d->cmdstr);
            ret=1; //this will return a '*' causing the host to fail
          }
           break;
      case 'F':  //fixed set of samples
	   //Dprintf("STRT_FIX\n\r");
           tx_init(d);
           d->cont=0;
           ret=0;
           break;
      case 'C':  //continous mode
            tx_init(d);
            d->cont=1;
            //Dprintf("STRT_CONT\n\r");
            ret=0;
            break;
      //format is Axyy where x is 0 for disabled, 1 for enabled and yy is channel #
      case 'A':  ///enable analog channel always a set
            tmpint=d->cmdstr[1]-'0'; //extract enable value
            tmpint2=atoi(&(d->cmdstr[2])); //extract channel number
            if((tmpint>=0)&&(tmpint<=1)&&(tmpint2>=0)&&(tmpint2<=31)){
               d->a_mask=d->a_mask & ~(1<<tmpint2);
               d->a_mask=d->a_mask | (tmpint<<tmpint2);
               //Dprintf("A%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->a_mask);
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
               //Dprintf("D%d EN %d Msk 0x%X\n\r",tmpint2,tmpint,d->d_mask);
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


