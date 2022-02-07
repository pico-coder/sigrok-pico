/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2018 mhooijboer <marchelh@gmail.com>
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
//debug print levels are err/warn/info/dbg/spew
#include <config.h>
#include <fcntl.h>
#include <glib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"


#define SERIALCOMM "115200/8n1"

static const uint32_t scanopts[] = {
        SR_CONF_CONN,  //Required OS name for the port, i.e. /dev/ttyACM0
	SR_CONF_SERIALCOMM, //Optional config of the port, i.e. 115200/8n1
};

//PulseView reads a sample rate config list as a min, max and step.
//If step is 1 then it creates a 1,2,5,10 set of selects, as well as the max.
//If step is not 1, then it gives a place to enter any value, which gives the greatest flexibility
static const uint64_t samplerates[] = {
        SR_HZ(10),
        SR_MHZ(120),
        SR_HZ(2),
};

static const uint32_t drvopts[] = {
	SR_CONF_OSCILLOSCOPE,
	SR_CONF_LOGIC_ANALYZER,
};
//SW trigger requires this
static const int32_t trigger_matches[] = {
        SR_TRIGGER_ZERO,
        SR_TRIGGER_ONE,
        SR_TRIGGER_RISING,
        SR_TRIGGER_FALLING,
        SR_TRIGGER_EDGE,
};


static const uint32_t devopts[] = {
//CLI prefers LIMIT_SAMPLES to be a list of high,low
        SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
        SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
//pulseview needs a list return to allow sample rate setting
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static struct sr_dev_driver sr_generic_driver_info;


static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
        struct sr_config *src;
        struct sr_dev_inst *sdi;
        struct sr_serial_dev_inst *serial;
	struct dev_context *devc;
	struct sr_channel *ch;
        GSList *l;
        int num_read;
        unsigned int i;
        const char *conn, *serialcomm;
        char buf[32];
	int len;
        uint8_t num_a,num_d,a_size;
	gchar *channel_name;
 
        conn = serialcomm = NULL;
        for (l = options; l; l = l->next) {
                src = l->data;
                switch (src->key) {
                case SR_CONF_CONN:
                        conn = g_variant_get_string(src->data, NULL);
                        break;
                case SR_CONF_SERIALCOMM:
                        serialcomm = g_variant_get_string(src->data, NULL);
                        break;
                }
        }
        if (!conn)
                return NULL;

        if (!serialcomm)
                serialcomm = SERIALCOMM;

        serial = sr_serial_dev_inst_new(conn, serialcomm);
        sr_info("Opening %s.", conn);
        if (serial_open(serial, SERIAL_RDWR) != SR_OK){
                sr_err("1st serial open fail");
                return NULL;
        }
        sr_info("Reseting device with *s at %s.", conn);
        send_serial_char(serial,'*');
        g_usleep(10000);
        //drain any inflight data
        do{
           sr_warn("Drain reads");
           len=serial_read_blocking(serial, buf,32,100);
           sr_warn("Drain reads done");
           if(len) sr_dbg("Dropping in flight serial data");
        }while(len>0);


        //Send identify 
        num_read=send_serial_w_resp(serial,"i\n",buf,20);
        if(num_read<16){
          sr_err("1st identify failed");
          serial_close(serial);
          g_usleep(100000);
          if (serial_open(serial, SERIAL_RDWR) != SR_OK){
                sr_err("2st serial open fail");
                return NULL;
          }
          g_usleep(100000);
          sr_err("Send second *");
          send_serial_char(serial,'*');
          g_usleep(100000);
          num_read=send_serial_w_resp(serial,"i\n",buf,20);
          if(num_read<10){
            sr_err("Second attempt failed");
            return NULL;
          }
        }
        //Expected ID response is SRGEN,AxxyDzz,VV 
        //where xx are number of analog channels, y is bytes per analog sample
        //and zz is number of digital channels, and VV is two digit version# which must be 00
        if((num_read<16)
           ||(strncmp(buf,"SRGEN,A",7))
           ||(buf[10]!='D')
           ||(buf[14]!='0') 
           ||(buf[15]!='0')){
           sr_err("ERROR:Bad response string %s %d",buf,num_read);
           return NULL;
        }
        a_size=buf[9]-'0';
        buf[9]='\0'; //Null to end the str for atois
        buf[13]='\0'; //Null to end the str for atois
        num_a=atoi(&buf[7]);
        num_d=atoi(&buf[11]);
        

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
        sdi->status = SR_ST_INACTIVE;
	sdi->vendor = g_strdup("Sigrok");
	sdi->model = g_strdup("Generic");
	sdi->version = g_strdup("00");
	sdi->conn = serial;
//broken/fixme
//	sdi->driver = &sr_generic_driver_info;
	sdi->inst_type = SR_INST_SERIAL;
	sdi->serial_num = g_strdup("N/A");
        if(((num_a==0)&&(num_d==0))
           ||(num_a>MAX_ANALOG_CHANNELS)
           ||(num_d>MAX_DIGITAL_CHANNELS)
           ||(a_size<1)
           ||(a_size>4)){
                sr_err("ERROR: invalid channel config a %d d %d asz %d",num_a,num_d,a_size);
		return NULL;
        }
	devc = g_malloc0(sizeof(struct dev_context));
	//TODO- deprecate a_size
        devc->a_size=a_size;
        //multiple bytes per analog sample not supported
        if((num_a>0)&&(devc->a_size!=1)){
          sr_err("Only Analog Size of 1 supported\n\r");
          return NULL;
        }
        devc->num_a_channels=num_a;
        devc->num_d_channels=num_d;
        devc->a_chan_mask=((1<<num_a)-1);
        devc->d_chan_mask=((1<<num_d)-1);
//The number of bytes that each digital sample in the buffers sent to the session. 
//All logical channels are packed together, where a slice of N channels takes roundup(N/8) bytes
//This never changes even if channels are disabled because PV expects disabled channels to still 
//be accounted for in the packing
//this ran forever but looks wrong... 16 channels would say 3 bytes
//        devc->dig_sample_bytes=((devc->num_d_channels/8)+1);
        devc->dig_sample_bytes=(devc->num_d_channels/8);
        if((devc->num_d_channels)&0x7) devc->dig_sample_bytes++;
	//These are the slice sizes of the data on the wire
        //1 7 bit field per byte
        devc->bytes_per_slice=(devc->num_a_channels*devc->a_size);
        if(devc->num_d_channels>0){
  	  // logic sent in groups of 7
          devc->bytes_per_slice+=(devc->num_d_channels+6)/7;
        }
        sr_dbg("num channels a %d d %d bps %d dsb %d",num_a,num_d,devc->bytes_per_slice,devc->dig_sample_bytes);
//Each analog channel is it's own grup
//Digital are just channels
//Grouping of channels is rather arbitrary as parameters like sample rate and number of samples
//apply to all changes.  Analog channels do have a scale and offset, but that it does
//without involvement of the session.
        devc->analog_groups = g_malloc0(sizeof(struct sr_channel_group *) *
                devc->num_a_channels);
	for (i = 0; i < devc->num_a_channels; i++) {
		channel_name = g_strdup_printf("A%d", i );
                                    //sdi, index, type, enabled,name
		ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG, TRUE, channel_name);
		devc->analog_groups[i] = g_malloc0(sizeof(struct sr_channel_group));
		devc->analog_groups[i]->name = channel_name;
		devc->analog_groups[i]->channels = g_slist_append(NULL, ch);
		sdi->channel_groups = g_slist_append(sdi->channel_groups,
			devc->analog_groups[i]);
	}

	if (devc->num_d_channels>0) {
                for (i = 0; i < devc->num_d_channels; i++){
			channel_name = g_strdup_printf("D%d", i);
                        sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE,
                                       channel_name);
			g_free(channel_name);
                }

	}
	//In large sample usages we get the call to receive with large transfers.
        //Since the CDC serial implemenation can silenty lose data as it gets close to full, allocate
        //storage for a half buffer which in a worst case scenario has 2x ratio of transmitted bytes to storage bytes
        devc->serial_buffer_size=256000;
        devc->buffer=NULL;
	sr_dbg("Setting serial buffer size: %i.", devc->serial_buffer_size);
        devc->cbuf_wrptr=0;
        //While slices are sent as a group of one sample across all channels, sigrok wants analog 
        //channel data sent as separate packets.  
        //Logical trace values are packed together.
        //A serial byte can never represent more than one sample, but allocate 2x the space anyway.
        devc->sample_buf_size=devc->serial_buffer_size*2;
        for(i=0;i<devc->num_a_channels;i++){
            devc->a_data_bufs[i]=NULL;
            devc->a_pretrig_bufs[i]=NULL;
        }
        devc->d_data_buf=NULL;
        devc->sample_rate=1000;        
        devc->capture_ratio=10;
        devc->rxstate=RX_IDLE;
	sdi->priv = devc;
        //Set an initial value as various code relies on an inital value.
        devc->limit_samples=1000;

//TODO - check return code,though I might just eliminate this function and add it's code here
        sr_generic_get_dev_cfg(sdi);
   
        sr_err("sr_err level logging enabled");
        sr_warn("sr_warn level logging enabled");
        sr_info("sr_info level logging enabled");
        sr_dbg("sr_dbg level logging enabled");
        sr_spew("sr_spew level logging enabled");
//TODO - ols closes the serial port here - should I?
        return std_scan_complete(di, g_slist_append(NULL, sdi));

}



//Note that on the initial driver load we pull all values into local storage.
//Thus gets can return local data, but sets have to issue commands to device.
static int config_set(uint32_t key, GVariant *data,
        const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
        struct dev_context *devc;
        int ret;
        (void)cg;
	if (!sdi)
		return SR_ERR_ARG;
        devc=sdi->priv;
        ret = SR_OK;
        sr_dbg("Got config_set key %d \n",key);
        switch (key) {
        case SR_CONF_SAMPLERATE:
                devc->sample_rate = g_variant_get_uint64(data);
                sr_dbg("config_set sr %llu\n",devc->sample_rate);
                break;
        case SR_CONF_LIMIT_SAMPLES:
                devc->limit_samples = g_variant_get_uint64(data);
                sr_dbg("config_set slimit %lld\n",devc->limit_samples);
                break;
        case SR_CONF_CAPTURE_RATIO:
                devc->capture_ratio = g_variant_get_uint64(data);
                break;

        default:
                sr_err("ERROR:config_set undefine %d\n",key);
                ret = SR_ERR_NA;
        }

        return ret;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
        sr_dbg("at config_get key %d",key);
        (void)cg;
	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->sample_rate);
                sr_spew("sample rate get of %lld",devc->sample_rate);
		break;
        case SR_CONF_CAPTURE_RATIO:
                if (!sdi)
                        return SR_ERR;
                devc = sdi->priv;
                *data = g_variant_new_uint64(devc->capture_ratio);
                break;
	case SR_CONF_LIMIT_SAMPLES:
                sr_spew("config_get limit_samples of %llu",devc->limit_samples);
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	default:
                sr_spew("unsupported cfg_get key %d",key);
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
        const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
        (void)cg;
        //scan or device options are the only ones that can be called without a defined instance
        if((key==SR_CONF_SCAN_OPTIONS)||(key==SR_CONF_DEVICE_OPTIONS)){
            return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
        }
        if (!sdi){
            sr_err("ERROR:\n\r\n\r\n\r Call to config list with null sdi\n\r\n\r");
            return SR_ERR_ARG;
       }
        sr_dbg("start config_list with key %X\n",key);
        switch(key){
//Pulseview in  pulseview/pv/toolbars/mainbar.cpp requires list support for frequencies as a triple
//of min,max,step.  If step is 1, then it proves a 1,2,5,10 select, but if not 1 it allows a spin box
        case SR_CONF_SAMPLERATE:
                sr_dbg("Return sample rate list");
                *data = std_gvar_samplerates_steps(ARRAY_AND_SIZE(samplerates));
                break;
//This must be set to get SW trigger support
        case SR_CONF_TRIGGER_MATCH:
                *data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
                break;
        case SR_CONF_LIMIT_SAMPLES:
                //Really this limit is up to the memory capacity of the host,
                //and users that pick huge values deserve what they get.
                //But setting this limit to prevent really crazy things.
                *data = std_gvar_tuple_u64(1LL,1000000000LL);
                sr_dbg("sr_config_list limit samples ");
                break;
        default:
               sr_dbg("reached default statement of config_list");

                return SR_ERR_NA;
        }

        return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct sr_serial_dev_inst *serial;
	struct dev_context *devc;
	struct sr_channel *ch;
        struct sr_trigger *trigger;
        char tmpstr[20];
	GSList *l;
        int a_enabled=0,d_enabled=0,len;
        //TODO - downgrade all the sr_err that are not errors once this works
	serial = sdi->conn;
        int i;
	devc = sdi->priv;
        sr_dbg("Enter acq start");
      sr_dbg("dsbstart %d",devc->dig_sample_bytes);
	devc->buffer = g_malloc(devc->serial_buffer_size);
        if(!(devc->buffer)){sr_err("ERROR:serial buffer malloc fail");return SR_ERR_MALLOC;}

//Get device in idle state
        if(serial_drain(serial)!=SR_OK){sr_err("Initial Drain Failed\n\r");return SR_ERR;}
        send_serial_char(serial,'*');
        if(serial_drain(serial)!=SR_OK){sr_err("Second Drain Failed\n\r");return SR_ERR;}
 
        sprintf(tmpstr,"L%lld\n", devc->limit_samples);
        if(send_serial_w_ack(serial, tmpstr)!=SR_OK) {
           sr_err("Sample limit to device failed");
           return SR_ERR;
        }
        sprintf(&tmpstr[0],"R%llu\n", devc->sample_rate);
        if(send_serial_w_ack(serial, tmpstr)!=SR_OK) {
           sr_err("Sample rate to device failed");
           return SR_ERR;
        }

        for (l = sdi->channels; l; l = l->next) {
         ch = l->data;
         sr_dbg("c %d enabled %d name %s\n",ch->index,ch->enabled,ch->name);

         if(ch->name[0]=='A'){
           devc->a_chan_mask&=~(1<<ch->index);
           if(ch->enabled) {
               devc->a_chan_mask|=(ch->enabled<<ch->index);
               a_enabled++;
           }
           sr_dbg("A%d en %d mask 0x%X",ch->index,ch->enabled,devc->a_chan_mask);
          
         }
         if(ch->name[0]=='D'){
           devc->d_chan_mask&=~(1<<ch->index);
           if(ch->enabled) {
              devc->d_chan_mask|=(ch->enabled<<ch->index);
              d_enabled++;
           }
           sr_dbg("D%d en %d mask 0x%X",ch->index,ch->enabled,devc->d_chan_mask);
         }
         sprintf(tmpstr,"%c%d%d\n",ch->name[0],ch->enabled,ch->index);
//TODO - do something with the error
         if (send_serial_w_ack(serial,tmpstr) != SR_OK){
            sr_err("ERROR:Channel enable fail");
   	 } else{
               }
       }//for all channels
       //recalculate bytes_per_slice.  
       devc->bytes_per_slice=(a_enabled*devc->a_size);

       for(i=0;i<devc->num_d_channels;i+=7){
	if(((devc->d_chan_mask)>>i)&(0x7F)){(devc->bytes_per_slice)++;}
       }

       sr_dbg("bps %d\n",devc->bytes_per_slice);
       devc->sent_samples=0;
       devc->byte_cnt=0;
       devc->bytes_avail=0;
       devc->wrptr=0; 
       devc->cbuf_wrptr=0;
       len=serial_read_blocking(serial, devc->buffer, devc->serial_buffer_size,serial_timeout(serial, 4));
       if(len>0){
          sr_info("Pre-ARM drain had %d characters:",len);
          devc->buffer[len]=0;
          sr_info("%s",devc->buffer);
       } 

        for(i=0;i<devc->num_a_channels;i++){
           devc->a_data_bufs[i]=g_malloc(devc->sample_buf_size*sizeof(float));
           if(!(devc->a_data_bufs[i])){sr_err("ERROR:analog buffer malloc fail");return SR_ERR_MALLOC;}
        }
        if(devc->num_d_channels>0){
          devc->d_data_buf=g_malloc(devc->sample_buf_size*devc->dig_sample_bytes);
          if(!(devc->d_data_buf)){sr_err("ERROR:logic buffer malloc fail");return SR_ERR_MALLOC;}
        }

//TODO-understand these params
       if ((trigger = sr_session_trigger_get(sdi->session))) {
              devc->pretrig_entries = (devc->capture_ratio * devc->limit_samples) / 100;
              devc->stl = soft_trigger_logic_new(sdi, trigger, devc->pretrig_entries);
              if (!devc->stl)
                   return SR_ERR_MALLOC;
              devc->trigger_fired=FALSE;
              if(devc->pretrig_entries>0){
              sr_dbg("Allocating pretrig buffers size %d",devc->pretrig_entries);
                for(i=0;i<devc->num_a_channels;i++){
                  if((devc->a_chan_mask>>i)&1){
//TODO-might look for the pointers being not null?
                      devc->a_pretrig_bufs[i] = g_malloc0(sizeof(float)*devc->pretrig_entries);
                      if(!devc->a_pretrig_bufs[i]){
                         sr_err("ERROR:Analog pretrigger buffer malloc failure, disabling");
                         devc->trigger_fired=TRUE;
                      }
                  }//if chan_mask
                 }//for num_a_channels
              }//if pre_trigger


              sr_info("Entering sw triggered mode");
              //post the receive before starting the device to ensure we are ready to receive data ASAP
//TODO-Jan/1 I had this at 7000 for scpi version but most serial uses are in the 50-200 range
//note that I call it twice (just below)
       	      serial_source_add(sdi->session, serial, G_IO_IN, 200,sr_generic_receive, (void *) sdi);
              sprintf(tmpstr,"C\n");
  	      if(send_serial_str(serial, tmpstr) != SR_OK)
		  return SR_ERR;

        } else{
              devc->trigger_fired=TRUE;
              devc->pretrig_entries=0;
              sr_info("Entering fixed sample mode");
       	      serial_source_add(sdi->session, serial, G_IO_IN, 200,sr_generic_receive, (void *) sdi);
              sprintf(tmpstr,"F\n");
  	      if(send_serial_str(serial,tmpstr) != SR_OK)
		  return SR_ERR;
        }
	std_session_send_df_header(sdi);

        sr_dbg("dsbstartend %d",devc->dig_sample_bytes);

        if(devc->trigger_fired) std_session_send_df_trigger(sdi);
	//Keep this at the end as we don't want to be RX_ACTIVE unless everything is ok
       devc->rxstate=RX_ACTIVE;

	return SR_OK;
}
//TODO-it's not clear how this is a clean stop if we are in the middle of an acquisition
//as the generic_receive may still be running and it's not clear how the device gets reinited
//part of the answer may be that we get a special event sent to the generic_receiver???
//though protocol.c references the enabled channels without checking for it becoming null..
//also need to review to make sure any mallocs in acquisition start are freed

//TODO - answer to above seems that we can hit this stop if the user hits stop in the pulseview app
//(perhaps a runaway sample), but we also need to always call this on the completion of any transfer
//because we free memory of the serail buffer and other things

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_serial_dev_inst *serial;
        sr_dbg("****at dev_acquisition_stop");
        int len;
 	devc = sdi->priv;
        serial = sdi->conn;

	std_session_send_df_end(sdi);
        //If we reached this while still active it is likely because the stop button was pushed in pulseview.
        //That normally should be some kind of error condition, so we don't try to check the bytenct
        if(devc->rxstate==RX_ACTIVE){
          sr_err("Reached dev_acquisition_stop in RX_ACTIVE");
          //In case we get calls to generic_receive force it to exit
          devc->rxstate=RX_IDLE;
	}

	//TODO- a reset '*' may be a bit too much, and the + likely is responded too quicker
        if(devc->rxstate!=RX_IDLE){
          sr_err("Sending plus to stop device stream\n\r");
          send_serial_char(serial,'+'); 
        }
        //drain data from device so that it doesn't confuse subsequent commands
        do{
           len=serial_read_blocking(serial, devc->buffer, devc->serial_buffer_size,100);
           if(len) sr_err("Dropping %d device bytes\n\r",len);
        }while(len>0);



	if(devc->buffer){g_free(devc->buffer);devc->buffer=NULL;}

        for(int i=0;i<devc->num_a_channels;i++){
           if(devc->a_data_bufs[i]){
              g_free(devc->a_data_bufs[i]);
              devc->a_data_bufs[i]=NULL;
           }
        }

        if(devc->d_data_buf){g_free(devc->d_data_buf);devc->d_data_buf=NULL;}
        for(int i=0;i<devc->num_a_channels;i++){
          if(devc->a_pretrig_bufs[i]) g_free(devc->a_pretrig_bufs[i]);
          devc->a_pretrig_bufs[i]=NULL;
        }

	serial= sdi->conn;
	serial_source_remove(sdi->session, serial);

	return SR_OK;
}

static struct sr_dev_driver sr_generic_driver_info = {
	.name = "sigrok-generic",
	.longname = "Sigrok Generic",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = std_serial_dev_open,
	.dev_close = std_serial_dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sr_generic_driver_info);
