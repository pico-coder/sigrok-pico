/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2016 mhooijboer <marchelh@gmail.com>
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE

#include <config.h>
#include <errno.h>
#include <glib.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

SR_PRIV int send_serial_str(struct sr_serial_dev_inst *serial, char *str){
        int len=strlen(str);
        if((len>15)||(len<1)){ //limit length to catch errant strings
           sr_err("ERROR:Serial string len %d invalid ",len);
           return SR_ERR;
        } 
        //100ms timeout. With USB CDC serial we can't define the timeout
        //based on link rate, so just pick something large as we shouldn't normally see them
        if(serial_write_blocking(serial,str,len,100) != len){
                sr_err("ERROR:Serial str write failed");
                return SR_ERR;
        }

        return SR_OK;
}
SR_PRIV int send_serial_char(struct sr_serial_dev_inst *serial, char ch){
        char buf[1];
        buf[0]=ch;
        if(serial_write_blocking(serial,buf,1,100) != 1){ //100ms
                sr_err("ERROR:Serial char write failed");
                return SR_ERR;
        }
        return SR_OK;
}   
//Issue a command that expects a string return, return length of string
int send_serial_w_resp(struct sr_serial_dev_inst *serial, char *str,char *resp,size_t cnt){
        int num_read,i;
        send_serial_str(serial,str);
       //Using the serial_read_blocking function when reading a response of unknown length requires 
       //a long worst case timeout to always be taken.  So, instead loop waiting for a first byte, and
       //then a final small delay for the rest. 
        for(i=0;i<1000;i++){ //wait up to 1 second in ms increments
            num_read = serial_read_blocking(serial, resp, cnt, 1);
            if(num_read>0) break;
        }
        //sr_spew("rwprsp1 i %d nr %d",i,num_read);
        //Since the serial port is usb CDC we can't calculate timeouts based on baud rate but
        //even if the response is split between two USB transfers 10ms should be plenty.
        num_read+= serial_read_blocking(serial, &(resp[num_read]), cnt-num_read, 10);
        //sr_spew("rwrsp2 nr %d",num_read);

        if ((num_read < 1)||(num_read>30)) {
                sr_err("ERROR:Serial_w_resp failed (%d).", num_read);
                return -1;
        }else{
         return num_read;
        }
}       
//Issue a command that expects a single char ack 
SR_PRIV int send_serial_w_ack(struct sr_serial_dev_inst *serial, char *str){
        char buf[2];
        int num_read;
        //In case we have left over transfer from the device, drain them
        while((num_read=serial_read_blocking(serial, buf, 2, 10))){
          //sr_dbg("swack drops 2 previous bytes %d %d",buf[0],buf[1]);
        }
        send_serial_str(serial,str);
        //1000ms timeout
        num_read =serial_read_blocking(serial, buf, 1, 1000);
        if ((num_read == 1)&&(buf[0]=='*')) {
                return SR_OK;
        }else{
                sr_err("ERROR:Serial_w_ack %s failed (%d).", str,num_read);
                if(num_read){
		  sr_err("ack resp char %c d %d\n\r",buf[0],buf[0]);
                }
                return SR_ERR;
        }
}        

//Process incoming data stream assuming it is optimized packing of 4 channels or less
//Each byte is 4 channels of data and a 3 bit rle value, or a larger rle value, or a control signal.
//This also checks for aborts and ends.
//If an end is seen we stop processing but do not check the byte_cnt
//The output is a set of samples fed to process group to perform sw triggering and sending of data to the session
//as well as maintenance of the serial rx byte cnt.
//Since we can get huge rle values we chop them up for processing into smaller groups
//In this mode we can always consume all bytes because there are no cases where the processing of one 
//byte requires the one after it.
void process_D4(struct sr_dev_inst *sdi,struct dev_context *d){
   int32_t j;
   uint8_t cbyte;
   uint8_t cval;
   uint32_t rlecnt=0;
   uint32_t sampcnt=0; //number of samples received with no rles
   while((d->ser_rdptr)<(d->bytes_avail)){
     cbyte=d->buffer[(d->ser_rdptr)];
     //RLE only byte
     if((cbyte>=48)&&(cbyte<=127)){ 
       rlecnt+=(cbyte-47)*8;
       d->byte_cnt++;
     }else if(cbyte>=0x80){ //sample with possible rle
       rlecnt+=(cbyte&0x70)>>4; 
       if(rlecnt){
         //Duplicate the previous values
         //The maximum value of one rle is 640 but we might have received several so
         //call process group first
	 //todo - the size of the digital buffer is much larger than this, but start with it for now...
         if((rlecnt+d->cbuf_wrptr)>2048){
	   //process_group is sent the number of slices which is just the cbufwrptr divided by the slice size
	   if((d->cbuf_wrptr)%(d->dig_sample_bytes)){
	     sr_err("Modulo fail %d %d ",d->cbuf_wrptr,d->dig_sample_bytes);
           }
           process_group(sdi,d,(d->cbuf_wrptr/d->dig_sample_bytes));
         }
	 rle_memset(d,rlecnt);
         rlecnt=0;
         sampcnt=0;
       }
       //Finally add in the new values
       cval=cbyte&0xF;
       d->d_data_buf[d->cbuf_wrptr++]=cval;
       //pad in all other bytes since the sessions even wants disabled channels reported
       for(j=1;j<d->dig_sample_bytes;j++){
	 d->d_data_buf[d->cbuf_wrptr++]=0;
       }
       sampcnt++;
       d->byte_cnt++;
       sr_spew("Dchan4 rdptr %d wrptr %d bytein 0x%X rle %d cval 0x%X\n",
              (d->ser_rdptr)-1,d->cbuf_wrptr,cbyte,rlecnt,cval);
       rlecnt=0;

       d->d_last[0]=cval;
     }
     //Any other character ends parsing - it could be a frame error or a start of the final byte cnt
     else {
       if(cbyte=='$'){
         sr_info("D4 Data stream stops with cbyte %d char %c rdidx %d cnt %llu",cbyte,cbyte,d->ser_rdptr,d->byte_cnt);
         d->rxstate=RX_STOPPED;
       }else{
         sr_err("D4 Data stream aborts with cbyte %d char %c rdidx %d cnt %llu",cbyte,cbyte,d->ser_rdptr,d->byte_cnt);
         d->rxstate=RX_ABORT;
       }
       break; //break from while loop
     }
     (d->ser_rdptr)++;
   }//while rdptr < wrptr
   sr_spew("D4 while done rdptr %d",d->ser_rdptr);
   //If we reach the end of the serial input stream send any remaining values or rles to the session
   /*this can also be skipped now the rle_memset handles cbufwrptr
   if(sampcnt){
     process_group(sdi,d,sampcnt);
     sampcnt=0;
    }   
   */
   if(rlecnt){
     sr_spew("Residual D4 slice rlecnt %d",rlecnt);
     rle_memset(d,rlecnt);
   }
   if(d->cbuf_wrptr){
     sr_spew("Residual D4 data wrptr %d",d->cbuf_wrptr);
     process_group(sdi,d,d->cbuf_wrptr/d->dig_sample_bytes);
     
   }

}//Process_D4

//Process incoming data stream and forward to trigger processing with process_group
//The final value of ser_rdptr indicates how many bytes were processed.
//This version handles all other enabled channel configurations that Process_D4 doesn't
void process_slice(struct sr_dev_inst *sdi,struct dev_context *devc){
   int32_t i;
   uint32_t tmp32;
   uint8_t cbyte;
   uint32_t slices_avail=0;
   uint32_t cword;
   uint32_t slice_bytes; //number of bytes that have legal slice values
   //Only process legal data values for this mode which are >=0x80
   for(slice_bytes=1;(slice_bytes<devc->bytes_avail)&&(devc->buffer[slice_bytes-1]>=0x80);slice_bytes++);
   if(slice_bytes!=devc->bytes_avail){
       cbyte=devc->buffer[slice_bytes-1];
       slice_bytes--; //Don't process the ending character
       if(cbyte=='$'){
         sr_info("Data stream stops with cbyte %d char %c rdidx %d sbytes %d cnt %llu",cbyte,cbyte,devc->ser_rdptr,slice_bytes,devc->byte_cnt);
         devc->rxstate=RX_STOPPED;
       }else{
         sr_err("Data stream aborts with cbyte %d char %c rdidx %d sbytes %d cnt %llu",cbyte,cbyte,devc->ser_rdptr,slice_bytes,devc->byte_cnt);
         devc->rxstate=RX_ABORT;
       }
   }
   //If the wrptr is non-zero due to a residual from the previous serial transfer don't double count it towards byte_cnt
   devc->byte_cnt+=slice_bytes-(devc->wrptr);
   sr_spew("process slice avail %d rdptr %d sb %d byte_cnt %d",devc->bytes_avail,devc->ser_rdptr,slice_bytes,devc->byte_cnt);
   //Must have a full slice
   while((devc->ser_rdptr+devc->bytes_per_slice)<=slice_bytes){
       //The use of devc->cbuf_wrptr is different between analog and digital.
       //For analog it targets a float sized offset for that channel's buffer 
       //For digital it targets a bit, so the 3 lsbs are bit offsets within a byte
       slices_avail++;
       cword=0;
       //build up a word 7 bits at a time, using only enabled channels
       for(i=0;i<devc->num_d_channels;i+=7){
         if(((devc->d_chan_mask)>>i)&0x7F){
           cword|=((devc->buffer[devc->ser_rdptr])&0x7F)<<i;
           (devc->ser_rdptr)++;
         }
       }
       //and then distribute 8 bits at a time to all possible channels
       for(i=0;i<devc->num_d_channels;i+=8){
         uint32_t idx=((devc->cbuf_wrptr)*devc->dig_sample_bytes)+(i>>3);
         devc->d_data_buf[idx]=cword&0xFF;
         sr_spew("Dchan i %d wrptr %d idx %d char 0x%X cword 0x%X",i,devc->cbuf_wrptr,idx,devc->d_data_buf[idx],cword);
         cword>>=8;
       }
       //Each analog value is a 7 bit value
       for(i=0;i<devc->num_a_channels;i++){
          if((devc->a_chan_mask>>i)&1){
              //a_size is depracted and must always be 1B
              tmp32=devc->buffer[devc->ser_rdptr]-0x80;
              devc->a_data_bufs[i][devc->cbuf_wrptr]=((float)tmp32 * devc->a_scale[i])+devc->a_offset[i];
              devc->a_last[i]=devc->a_data_bufs[i][devc->cbuf_wrptr];
              sr_spew("AChan %d value %f ",i,devc->a_data_bufs[i][devc->cbuf_wrptr]);
              devc->ser_rdptr++;
           }//if channel enabled
       }//for num_a_channels
       devc->cbuf_wrptr++;
   }//While another slice available
   if(slices_avail){
     process_group(sdi,devc,slices_avail);
   }

}
//Send the processed analog values to the session
int send_analog(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_samples, uint32_t offset){
   struct sr_datafeed_packet packet;
   struct sr_datafeed_analog analog;
   struct sr_analog_encoding encoding;
   struct sr_analog_meaning meaning;
   struct sr_analog_spec spec;
   struct sr_channel *ch;
   uint32_t i;
   float *fptr;

   sr_analog_init(&analog, &encoding, &meaning, &spec, ANALOG_DIGITS);
   for(i=0;i<devc->num_a_channels;i++){
       if((devc->a_chan_mask>>i)&1){
          ch=devc->analog_groups[i]->channels->data;
          analog.meaning->channels = g_slist_append(NULL, ch);
          analog.num_samples = num_samples;
          analog.data = (devc->a_data_bufs[i]) + offset;
          fptr=analog.data;
          sr_spew("send analog num %d offset %d first %f 2 %f",num_samples,offset,*(devc->a_data_bufs[i]),*fptr);
          analog.meaning->mq = SR_MQ_VOLTAGE;
          analog.meaning->unit = SR_UNIT_VOLT;
          analog.meaning->mqflags = 0;
          packet.type = SR_DF_ANALOG;
          packet.payload = &analog;
          sr_session_send(sdi, &packet);
          g_slist_free(analog.meaning->channels);
       } //if enabled
   }//for channels
   return 0;

}
//Send the ring buffer of pre-trigger analog samples.
//  The entire buffer is sent (as long as it filled once), but need send two payloads split at the 
//  the writeptr 
int send_analog_ring(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_samples){
   struct sr_datafeed_packet packet;
   struct sr_datafeed_analog analog;
   struct sr_analog_encoding encoding;
   struct sr_analog_meaning meaning;
   struct sr_analog_spec spec;
   struct sr_channel *ch;
   int i;
   uint32_t num_pre,start_pre;
   uint32_t num_post,start_post;
   num_pre=(num_samples>=devc->pretrig_wr_ptr) ? devc->pretrig_wr_ptr : num_samples;
   start_pre=devc->pretrig_wr_ptr-num_pre;
   num_post=num_samples-num_pre;
   start_post=devc->pretrig_entries-num_post;
   sr_spew("send_analog ring wrptr %u ns %d npre %u spre %u npost %u spost %u",devc->pretrig_wr_ptr,num_samples,num_pre,start_pre,num_post,start_post);
   float *fptr;
   sr_analog_init(&analog, &encoding, &meaning, &spec, ANALOG_DIGITS);
   for(i=0;i<devc->num_a_channels;i++){
       if((devc->a_chan_mask>>i)&1){
          ch=devc->analog_groups[i]->channels->data;
          analog.meaning->channels = g_slist_append(NULL, ch);
          analog.meaning->mq = SR_MQ_VOLTAGE;
          analog.meaning->unit = SR_UNIT_VOLT;
          analog.meaning->mqflags = 0;
          packet.type = SR_DF_ANALOG;
          packet.payload = &analog;
         //First send what is after the write pointer because it is oldest
          if(num_post){
            analog.num_samples = num_post;
            analog.data = (devc->a_pretrig_bufs[i]) + start_post;
	    //sr_spew("ring buf %d starts at %p",i,(void *) devc->a_pretrig_bufs[i]);
            //sr_spew("analog data %d starts at %p",i,(void *) analog.data);
            //sr_spew("Sending A%d ring buffer oldest ",i);
            for(uint32_t j=0;j<analog.num_samples;j++){
             fptr=analog.data+(j*sizeof(float));
             //sr_spew("RNGDCT%d j %d %f %p",i,j,*fptr,(void *)fptr);
            }
            sr_session_send(sdi, &packet);
          }
          if(num_pre){
            analog.num_samples = num_pre;
            analog.data = (devc->a_pretrig_bufs[i])+start_pre;
            sr_dbg("Sending A%d ring buffer newest ",i);
            for(uint32_t j=0;j<analog.num_samples;j++){
               fptr=analog.data+(j*sizeof(float));
               sr_spew("RNGDCW%d j %d %f %p",i,j,*fptr,(void *)fptr);
            }
            sr_session_send(sdi, &packet);
          }    
          g_slist_free(analog.meaning->channels);
          sr_dbg("Sending A%d ring buffer done ",i);
       } //if enabled
   }//for channels
   return 0;

}

//Given a chunk of slices forward to trigger check or session as appropriate and update state
//these could be real slices or those generated by rles
int process_group(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_slices){
     int trigger_offset;
     int pre_trigger_samples;
     //These are samples sent to session and are less than num_slices if we reach limit_samples
     size_t num_samples;
     struct sr_datafeed_logic logic;
     struct sr_datafeed_packet packet;
     int i;
     size_t cbuf_wrptr_cpy;
     cbuf_wrptr_cpy=devc->cbuf_wrptr;
     //regardless of whether we forward samples on or not (because we aren't triggered), always reset the 
     //pointer into the device data buffers 
     devc->cbuf_wrptr=0;
     if(devc->trigger_fired){ //send directly to session
          if (devc->limit_samples &&
                 num_slices > devc->limit_samples - devc->sent_samples){
                      num_samples = devc->limit_samples - devc->sent_samples;   
         }else{
            num_samples=num_slices;
         }
         if(num_samples>0) {
           sr_spew("Process_group sending %d post trig samples dsb %d",num_samples,devc->dig_sample_bytes);
           //for(int z=0;(z<num_samples);z+=2){
           //  sr_spew("0x%X ",devc->d_data_buf[z]);
           //}
           if(devc->num_d_channels){
             packet.type = SR_DF_LOGIC;
             packet.payload = &logic;
             //Size the number of bytes required to fit all of the channels
             logic.unitsize = devc->dig_sample_bytes;  
             //The total length of the array sent
             logic.length=num_samples*logic.unitsize;
             logic.data = devc->d_data_buf;
             sr_session_send(sdi, &packet);
           }
           send_analog(sdi,devc,num_samples,0);
        }//num_sample>0
        devc->sent_samples+=num_samples;
        return 0;
     } //trigger_fired
     else{
        size_t num_ring_samples;
        size_t sptr;
        size_t eptr;
        size_t numtail;
        size_t numwrap;
        size_t srcptr;
        //sr_spew("Process_group check %d pre trig samples",num_slices);
        //The trigger_offset is -1 if no trigger is found, but if a trigger is found
        //then trigger_offset is the offset into the data buffer sent to it.
        //The pre_trigger_samples is the total number of samples before the trigger, but limited to
        //the size of the ring buffer set by the capture_ratio. So the pre_trigger_samples can include both the new samples
        //and the ring buffer, but trigger_offset is only in relation to the new samples
        trigger_offset = soft_trigger_logic_check(devc->stl,
                devc->d_data_buf, num_slices * devc->dig_sample_bytes, &pre_trigger_samples);
       //A trigger offset >=0 indicate a trigger was seen.  The stl will isue the trigger to the session
       //and will forward all pre trigger logic samples, but we must send any post trigger logic 
       //and all pre and post trigger analog signals
       // sr_dbg("trggr_off %d",trigger_offset);
       // sr_dbg("pre_samp  %d",pre_trigger_samples);
        if (trigger_offset > -1) {
                devc->trigger_fired = TRUE;
                devc->sent_samples += pre_trigger_samples;
                packet.type = SR_DF_LOGIC;
                packet.payload = &logic;
                num_samples = num_slices - trigger_offset;
//Since we are in continuous mode for SW triggers it is possible to get more samples than limit_samples, so
//once the trigger fires make sure we don't get beyond limit samples. At this point sent_samples should
//be equal to pre_trigger_samples (just added above) because without being triggered we'd never increment
//sent_samples.
//This number is the number of post trigger logic samples to send to the session, the number of floats
//is larger because of the analog ring buffer we track.
                if (devc->limit_samples &&
                   num_samples > devc->limit_samples - devc->sent_samples)
                      num_samples = devc->limit_samples - devc->sent_samples;
               //The soft trigger logic issues the trigger and sends packest for all logic data that was pretrigger
               //so only send what is left
                if(num_samples>0){
                   sr_dbg("Sending post trigger logical remainder of %d",num_samples);
                   logic.length = num_samples * devc->dig_sample_bytes;
                   logic.unitsize = devc->dig_sample_bytes;
                   logic.data = devc->d_data_buf + (trigger_offset * devc->dig_sample_bytes);
                   devc->sent_samples += num_samples;
                   sr_session_send(sdi, &packet);
                }
                size_t new_start,new_end,new_samples,ring_samples;
                //Figure out the analog data to send.
                //We might need to send:
                //-some or all of incoming data
                //-all of incoming data and some of ring buffer
                //-all of incoming data and all of ring buffer (and still might be short)
                //We don't need to compare to limit_samples because pretrig_entries can never be more than limit_samples
                //trigger offset indicatese where in the new samples the trigger was, but we need to go back pretrig_entries before it             
                new_start=(trigger_offset>(int)devc->pretrig_entries) ? trigger_offset-devc->pretrig_entries : 0;
               //Note that we might not have gotten all the pre triggerstore data we were looking for. In such a case the sw trigger
               //logic seems to fill up to the limit_samples and thus the ratio is off, but we get the full number of samples
               //The number of entries in the ring buffer is pre_trigger_samples-trigger_offset so subtract that from limit samples
               //as a threshold
                new_end=MIN(num_slices-1,devc->limit_samples-(pre_trigger_samples-trigger_offset)-1);
               //This includes pre and post trigger storage.
                new_samples=new_end-new_start+1;
               //pre_trigger_samples can never be greater than trigger_offset by more than the ring buffer depth (pretrig entries) 
                ring_samples=(pre_trigger_samples>trigger_offset) ? pre_trigger_samples-trigger_offset : 0;
                sr_spew("SW trigger float info newstart %zu new_end %zu new_samp %zu ring_samp %zu",new_start,new_end,new_samples,ring_samples);
                if(ring_samples>0){
                   send_analog_ring(sdi,devc,ring_samples);
                }
                if(new_samples){
                  send_analog(sdi,devc,new_samples,new_start);
                }

         }//if trigger_offset 
          else { //We didn't trigger but need to copy to ring buffer
            if((devc->a_chan_mask)&&(devc->pretrig_entries)){
             //The incoming data buffer could be much larger than the ring buffer, so never copy more than 
             //the size of the ring buffer
             num_ring_samples=num_slices > devc->pretrig_entries ? devc->pretrig_entries : num_slices;
             sptr=devc->pretrig_wr_ptr;  //starting pointer to copy to
             //endptr can't go past the end
             eptr=(sptr+num_ring_samples)>=devc->pretrig_entries ? devc->pretrig_entries-1 : sptr+num_ring_samples-1;
             numtail=(eptr-sptr)+1; //number of samples to copy to the tail of ring buffer without wrapping
             numwrap=(num_ring_samples>numtail) ? num_ring_samples-numtail:0;
             //cbuf_wrptr points to where the next write should go, not  theactual write data
             srcptr=cbuf_wrptr_cpy-num_ring_samples;
             sr_spew("RNG num %zu sptr %zu eptr %zu ",num_ring_samples,sptr,eptr);
             //sr_spew("RNG srcptr %zu nt %zu nw %zu",srcptr,numtail,numwrap);
             for(i=0;i<devc->num_a_channels;i++){
               if((devc->a_chan_mask>>i)&1){
                 //copy tail
                 for(uint32_t j=0;j<numtail;j++){
                   devc->a_pretrig_bufs[i][sptr+j]=devc->a_data_bufs[i][srcptr+j];
                   //sr_spew("RNGCpyT C%d src %zu dest %zu",i,srcptr+j,sptr+j);
                 }//for j
                 } //if chan_mask
             }//for channels
            //Copy wrap
            srcptr+=numtail;
            for(i=0;i<devc->num_a_channels;i++){
               if((devc->a_chan_mask>>i)&1){
               for(uint32_t j=0;j<numwrap;j++){
                 devc->a_pretrig_bufs[i][j]=devc->a_data_bufs[i][srcptr+j];
                 //sr_spew("RNGCpyW C%d src %zu dest %zu",i,srcptr+j,j);
               }//for j
              }//if chan_mask
             }//for channels
           devc->pretrig_wr_ptr=(numwrap) ? numwrap : (eptr+1)%devc->pretrig_entries;
           //sr_dbg("RNG pwrptr new %u",devc->pretrig_wr_ptr);
          }//if any analog channel enabled and pretrig_entries
      }//else (trigger not detected)
     }//trigger not set on function entry
      return 0;
}//process_group


//Duplicate previous sample values
//This function relies on the caller to ensure d_data_buf has samples to handle the full value of the rle
void rle_memset(struct dev_context *devc,uint32_t num_slices){
      uint32_t j,k;
      sr_spew("rle_memset val 0x%X,slices %d dsb %ld\n",devc->d_last[0],num_slices,devc->dig_sample_bytes);
      //Even if a channel is disabled, PV expects the same location and size for the enabled
      // channels as if the channel were enabled.
      for(j=0;j<num_slices;j++){
        for(k=0;k<devc->dig_sample_bytes;k++){
           devc->d_data_buf[devc->cbuf_wrptr++]=devc->d_last[k];
           //sr_spew("k %d j %d v 0x%X",k,j,devc->d_data_buf[(devc->cbuf_wrptr)-1]);
        }
      }
}

//This callback function is mapped from api.c with serial_source_add and is created after a capture
//has been setup and is responsible for querying the device trigger status, downloading data
//and forwarding packets
SR_PRIV int raspberrypi_pico_receive(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
        struct sr_serial_dev_inst *serial;
	uint32_t i;
        int len;
        uint32_t bytes_rem;
        uint32_t residual_bytes;
	(void)fd;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;
        if(devc->rxstate!=RX_ACTIVE){
          //This condition is normal operation and expected to happen 
          //but printed as information
	  sr_dbg("Reached non active state in receive %d",devc->rxstate);
	  //don't return - we may be waiting for a final bytecnt
          //return TRUE;
        }
        if(devc->rxstate==RX_IDLE){
          //This is the normal end condition where we do one more receive
          //to make sure we get the full byte_cnt
	  sr_dbg("Reached idle state in receive %d",devc->rxstate);
          return FALSE;
        }

        serial = sdi->conn;
        //return true if it is some kind of event we don't handle
        if (!(revents == G_IO_IN || revents == 0))
                return TRUE;
        //Fill the buffer, note the end may have partial slices
        bytes_rem=devc->serial_buffer_size - devc->wrptr;  
        //Read one byte less so that we can null it and print as a string
        //Do a small 10ms timeout, if we get nothing, we'll always come back again
        len=serial_read_blocking(serial, &(devc->buffer[devc->wrptr]), bytes_rem-1,10);
        sr_spew("Entry wrptr %u bytes_rem %u len %d",devc->wrptr,bytes_rem,len);

        if(len>0){
           devc->buffer[devc->wrptr+len]=0;
           //Add the "#" so that spaces are clearly seen
           sr_dbg("rx string %s#",devc->buffer);
           //This is not guaranteed to be a dataloss condition, but definitely indicates we are 
           //processing data right at the incoming rate.
           //With the addition of the byte_cnt sent at the end we will detect any dataloss conditions
           //and thus this is disabled
           //if(len>=(int)bytes_rem-8){
           //  sr_err("ERROR: Serial buffer near or at max depth, data from device may have been lost");
           //}
           devc->bytes_avail=(devc->wrptr+len);
           sr_spew("rx len %d bytes_avail %ul sent_samples %ul wrptr %u",len,devc->bytes_avail,devc->sent_samples,devc->wrptr);
           //sr_err("rx len %d ",len);
        }else if (len==0){
           return TRUE;
        }else {
           sr_err("ERROR:Negative serial read code %d",len);
           sdi->driver->dev_acquisition_stop(sdi);
           return FALSE;
        }//len>0
        //This can be used as a bit bucket to drop all samples to see how host processing time effects
        //the devices ability to send data. Obviously no data will be forwarded to the session so it will hang
	//	return TRUE; 

        //Process the serial read data
        devc->ser_rdptr=0; 
        if(devc->rxstate==RX_ACTIVE){
	  if((devc->a_chan_mask==0)&&((devc->d_chan_mask&0xFFFFFFF0)==0)){
           process_D4(sdi,devc);
          }else{
           process_slice(sdi,devc);
          }
        }
        //process_slice/process_D4 increment ser_rdptr as bytes of the serial buffer are used
        //But they may not use all of it, and thus the residual unused bytes are shifted to the start of the buffer
        //for the next call.
        residual_bytes=devc->bytes_avail - devc->ser_rdptr;
        //sr_spew("Residuals resid %d avail %d rdptr %d wrptr %d\n",residual_bytes,devc->bytes_avail,devc->ser_rdptr,devc->wrptr);
        if(residual_bytes){
           for(i=0;i<residual_bytes;i++){
             devc->buffer[i]=devc->buffer[i+devc->ser_rdptr];
           }
           devc->ser_rdptr=0;
           devc->wrptr=residual_bytes;
           sr_spew("Residual shift rdptr %u wrptr %u",devc->ser_rdptr,devc->wrptr);
        }else{
          //If there are no residuals shifted then zero the wrptr since all data is used
           devc->wrptr=0;
        }
        //ABORT ends immediately
        if(devc->rxstate==RX_ABORT){
              sr_err("Ending receive on abort");
	      sdi->driver->dev_acquisition_stop(sdi);
              return FALSE;//
        }
        //if stopped look for final '+' indicating the full byte_cnt is received
        if(devc->rxstate==RX_STOPPED){
            sr_dbg("Stopped, checking byte_cnt");
            if(devc->buffer[0]!='$'){
              //If this happens it means that we got a set of data that was not processed as
              //whole groups of slice bytes. So either we lost data or are not parsing it correctly.
               sr_err("ERROR: Stop marker should be byte zero");
               devc->rxstate=RX_ABORT;
  	       sdi->driver->dev_acquisition_stop(sdi);
               return FALSE;
            }
            for(i=1;i<devc->wrptr;i++){
               if(devc->buffer[i]=='+'){
                  devc->buffer[i]=0;
                  uint64_t rxbytecnt;
                  rxbytecnt=atol(&(devc->buffer[1]));
                  sr_dbg("Byte_cnt check device cnt %llu host cnt %llu",rxbytecnt,devc->byte_cnt);
                  if(rxbytecnt!=devc->byte_cnt){
		    sr_err("ERROR: received %llu and counted %llu bytecnts don't match, data may be lost",rxbytecnt,devc->byte_cnt);
                  } 
                  //Since we got the bytecnt we know the device is done sending data
                  devc->rxstate=RX_IDLE;
		  //We must always call acquisition_stop on all completed runs
		  sdi->driver->dev_acquisition_stop(sdi);
                  return TRUE;
               }
            }
            //It's possible we need one more serial transfer to get the byte_cnt, so print that here
            sr_dbg("Haven't seen byte_cnt + yet");
        }//RX_STOPPED
        //If at the sample limit, send a "+" in case we are in continuous mode and need
        //to stop the device.  Not that even in non continous mode there might be cases where get an extra
        //sample or two...

        if((devc->sent_samples>=devc->limit_samples)&&(devc->rxstate==RX_ACTIVE)){ 
           sr_dbg("Ending: sent %u of limit %llu samples byte_cnt %llu",
                   devc->sent_samples,devc->limit_samples,devc->byte_cnt);
           send_serial_char(serial,'+'); 

        }
        sr_spew("Receive function done: sent %u limit %llu wrptr %u len %d",devc->sent_samples,devc->limit_samples,devc->wrptr,len);
	return TRUE;
}//raspberrypi_pico_receive

//Read device specific information from the device
SR_PRIV int raspberrypi_pico_get_dev_cfg(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
        struct sr_serial_dev_inst *serial;
	char *cmd, response[20];
        gchar **tokens;
	unsigned int i;
	int ret,num_tokens;

	devc = sdi->priv;
        sr_dbg("At get_dev_cfg");
        serial = sdi->conn;
        for(i=0;i<devc->num_a_channels;i++){
           cmd = g_strdup_printf("a%d\n",i);
           ret = send_serial_w_resp(serial,cmd,response,20);
           if(ret<=0){
              sr_err("ERROR:No response from device for analog channel query");
              return SR_ERR;
           }
           tokens=NULL;
           tokens = g_strsplit(response, ",", 0);
           num_tokens = g_strv_length(tokens);
           if (num_tokens == 2) {
             devc->a_scale[i]=atof(tokens[0]);
             devc->a_offset[i]=atof(tokens[1]);
             sr_dbg("A%d scale %f offset %f\n",i,devc->a_scale[i],devc->a_offset[i]);
           }else{
             sr_err("ERROR:Ascale read c%d got unparseable response %s",i,response);
             //force a good fixed value
             devc->a_scale[i]=1/256;
             devc->a_offset[i]=0;
           }
           g_strfreev(tokens);
           g_free(cmd);
        }


        return SR_OK;

}

