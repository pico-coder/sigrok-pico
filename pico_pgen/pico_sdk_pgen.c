//simple generator
//Mask of bits 22:2 to use as outputs - 
#define GPIO_D_MASK 0x7FFFFC

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stdarg.h"
#include <string.h>


int main(){

    long int i,j;

//    set_sys_clock_khz(250000,true);
//    uart_set_format(uart0,8,1,1);
//    uart_init(uart0,115200);
    stdio_init_all();
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);   
    sleep_us(100000);    
    printf("\n\n\nhello from pgen \r\n\n\n");
    //Use 22:2 as digital
    gpio_init_mask(GPIO_D_MASK); //set as GPIO_FUNC_SIO and clear output enable
    gpio_set_dir_masked(GPIO_D_MASK,GPIO_D_MASK);  //TODO - I think an out can still be an ind?
    uint32_t ctime,tstart;
    uint32_t cval;
    tstart=time_us_32();
    while(true){
     ctime=time_us_32();
     // shift up/down to test performance with rle etc
     //cval=(ctime-tstart)<<4; //not a true  62.5ns/8Mhz wave, but will increment as such
     //cval=(ctime-tstart)<<2; //not a true 250ns/2Mhz wave, but will increment as such
     //cval=(ctime-tstart); //1us, 500kz
     //cval=(ctime-tstart)>>1; //2us, 256khz
     //cval=(ctime-tstart)>>3; //8us, 64khz
     cval=(ctime-tstart)>>5; //32us, 16khz
     //cval=(ctime-tstart)>>6; //64us, 8khz
     //cval=(ctime-tstart)>>7; //128us, 4khz
     //cval=(ctime-tstart)>>9; //512us, 1khz
     //cval=(ctime-tstart)>>11; //2ms, 250hz
     //cval=0x123456789;
     //Note: both the mask and the value must be shifted up by the starting number of bits
     gpio_put_masked(GPIO_D_MASK,cval<<2); 
     //printf("cval 0x%X",cval);
   }//while true

 

}//main
