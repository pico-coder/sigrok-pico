//1st version to communicate over sigrok generic
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "stdarg.h"
#include <string.h>
const uint LED_PIN = 25;
#define MAX_FMT_SIZE 256
#define MAX_SAMPLE_RATE 125000000
#define MAX_SAMPLES 100000

int Dprintf(const char *fmt, ...)
{
    va_list argptr;
    int len = 1;
    char    _dstr[256];
   

     memset(&_dstr, 0x00, sizeof(_dstr) );
    va_start(argptr, fmt);
    len = vsprintf(_dstr, fmt, argptr);
    va_end(argptr);

        if ( (len > 0) && (len < 256 ) )
        {

    uart_puts(uart0,_dstr);
    uart_puts(uart0,"\r\n");
        }
    return len;
}
int main(){
    char cmdstr[20];
    int cmdstrptr=0;
    char charin,tmpchar;
    int intin;
    long int intin2,i,j,idx;
    int cnt=0;
    int armed=0;
    int num_a=3,num_d=2;
    long int sample_rate=125000000; //max sample rate is initial value
    unsigned long int scnt,num_samples=5;
    char tmpstr[100],a_bytes=3;
    long int tmpint,tmpint2;
    int res;
    uint64_t starttime,endtime;
//    sleep_ms(1000); //might help stability???
    stdio_usb_init();
//    sleep_ms(1000);
    uart_set_format(uart0,8,1,1);
    intin=uart_init(uart0,115200);
    
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);   
//doesn't seem to help stabiliyt    sleep_ms(3000);
    Dprintf("hello from loggerport \r\n");
    intin=0;
    while(1){
       intin=getchar_timeout_us(0);//000);
       if(intin>=0){  //PICO_ERROR_TIMEOUT=-1
           charin=(char)intin;
//TODO - SCPI may only send the \n, not \r...
           if((charin=='\r')||(charin=='\n')){
              cmdstr[cmdstrptr]=0;
              Dprintf("Cmd %s",cmdstr);
              if(!strcmp(cmdstr,"*IDN?")){
//AxxxyDzzz - num analog, analog size, num digital
//24 characters plus newline
//                 printf("SR_GENERIC,A0023D002,0,0\n");
                 printf("SR_GENERIC,A%03d%dD%03d,0,0\n",num_a,a_bytes,num_d);
                 Dprintf("sent IDN response");
              }
              else if(!strncmp(cmdstr,"SRT",3)){
                  if(cmdstr[3]=='?'){
                    printf("%d\n",sample_rate);
                    Dprintf("smp rate rsp %d",sample_rate);
                  }else{
                    tmpint=atoi(&cmdstr[3]);
                    if((tmpint>0)&&(tmpint<=MAX_SAMPLE_RATE)){
                      sample_rate=tmpint;
                      Dprintf("set smp rate to %d",sample_rate);
                    }else{
                      Dprintf("unsupported smp rate %s",cmdstr);
                    }
                 }
              }
              else if(!strncmp(cmdstr,"SMP",3)){
                 if(cmdstr[3]=='?'){
                    printf("%d\n",num_samples);
                    Dprintf("num smp rate rsp %d",num_samples);
                 }else{
                   tmpint=atoi(&cmdstr[3]);
                   if((tmpint>0)&&(tmpint<=MAX_SAMPLES)){
                      num_samples=tmpint;
                      Dprintf("set num smp to %d",num_samples);
                   }else{
                      Dprintf("unsupported num samples %s",cmdstr);
                  }
                 }
              }
              else if(!strncmp(cmdstr,"ARM",3)){
                 armed=1;
                 scnt=0;
                 Dprintf("armed");
                 starttime=time_us_64();//get_absolute_time();
              }
              else if(!strncmp(cmdstr,"STP",3)){
                 //TODO -send end of capture  if armed...              
                 armed=0;
                 scnt=0;
                 Dprintf("stopped"); 
              }
              cmdstrptr=0;

           }else{
//              sprintf(tmpstr,"newchar %c %d \r\n",charin,intin);
//              uart_puts(uart0,tmpstr);
              if(cmdstrptr>=19){
                cmdstrptr=0;
                Dprintf("Command overflow");
              }
              cmdstr[cmdstrptr++]=charin;
 
           }
       }//intin>=0
//I modified the pico sdk to add a puts_raw_len function so that I could send byte patterns of fixed length
//that were not c strings.  But, going to use ascii compliant values to avoid serial configuration issues
//and thus can end a set of bytes with a null to make it a string
//     puts_raw_len("0000000000111111111122222222223333333333",40);
//     scnt+=4;

   if(armed&&(scnt<num_samples)){
       idx=0;
       for(i=0;i<num_a;i++){
         //calculate an N byte value to match up with analog size
         tmpint=((scnt+i)*10000)&((1<<(8*a_bytes))-1);
    //     Dprintf("A%d val %d",i,tmpint);
         for(j=0;j<a_bytes;j++){
           //get one byte - least sig byte first
           tmpint2=(tmpint>>(j*8))&0xFF;
//           Dprintf("tmpint2 %d",tmpint2);
           //convert to ascii mode - lowest nibble first
           tmpstr[idx++]=((char)(tmpint2&0xF))+'0';
           tmpstr[idx++]=((char)((tmpint2>>4)&0xF))+'0';
 //          Dprintf("chars %c %c",tmpstr[idx-2],tmpstr[idx-1]);
         }
       } //for num_a
       for(i=0;i<num_d;i+=4){
       //todo this could use better masking based on num channels, but will help find bugs if
       //we use bits we shouldnt.  Since we send one char per 4 bits
         tmpint=(scnt>>i)&0xf;
   //      Dprintf("D%d val %d",i,tmpint);
         tmpstr[idx++]=(tmpint&0xF)+'0';
    //     Dprintf("chars %c ",tmpstr[idx-1]);
       }
       tmpstr[idx]=0;
//       puts_raw(tmpstr);
       printf("%s",tmpstr);
       Dprintf("Slice string %s",tmpstr);
       scnt++;
       if(scnt==num_samples){
           armed=0;
           endtime=time_us_64();
           Dprintf("Total time %llu %llu %llu",(endtime-starttime),endtime,starttime);
       }
   } //if armed
   }//while(1)
 

}//main
