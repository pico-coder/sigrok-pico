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

#ifndef LIBSIGROK_HARDWARE_RASPBERRYPI_PICO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_RASPBERRYPI_PICO_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

//This is used by sr_dbg/log etc
#define LOG_PREFIX "srgn"
 //number of bytes between markers
#define MRK_STRIDE 128

//This must be 32 or or less since many channel enable/disable masks and other elements may be only 32 bits wide.
//But is reduced further based on pico board limitations
#define MAX_ANALOG_CHANNELS 3
#define MAX_DIGITAL_CHANNELS 21
//digits input to sr_analog_init
#define ANALOG_DIGITS 4

SR_PRIV int send_serial_str(struct sr_serial_dev_inst *serial, char *str);
SR_PRIV int send_serial_char(struct sr_serial_dev_inst *serial, char ch);
int send_serial_w_resp(struct sr_serial_dev_inst *serial, char *str,char *resp,size_t cnt);
SR_PRIV int send_serial_w_ack(struct sr_serial_dev_inst *serial, char *str);

typedef enum rxstate {
  RX_IDLE=0,//not receiving
  RX_ACTIVE=1, //receiving data
  RX_STOPPED=2, //received stop marker, waiting for byte cnt
  RX_ABORT=3, //received aborted marker or other error
}rxstate_t;
//TODO todo - stopped review here - renam wrptr, and review all variables
struct dev_context {
/*Configuration Parameters */
        //It is up to the user to understand sample rates and serial download speed etc and 
        // do the right thing. i.e. don't expect continuous streaming bandwidth greater 
        //than serial link speed etc...
        //The number of samples the user expects to see.
        uint64_t limit_samples;
	uint64_t sample_rate;
        //Number of samples that have been received and processed
	uint32_t num_samples;   
        //Initial Number of analog and digital channels.  
        //This is set by initial device config.  Channels can be disabled/enabled, 
        //but can not be added/removed once driver is loaded. 
        uint16_t num_a_channels;
        uint16_t num_d_channels;
        //Masks of enabled channels based on user input
        uint32_t a_chan_mask;
        uint32_t d_chan_mask;
        // Channel groups -each analog channel is it's own group
        struct sr_channel_group **analog_groups;
        struct sr_channel_group *digital_group;
        //Data size in bytes for each analog channel in bytes 
        //must be 1 as only single byte samples are supported in this version
        uint8_t a_size;
        //Offset and scale for each analog channel to covert bytes to float
	float a_offset[MAX_ANALOG_CHANNELS];
 	float a_scale[MAX_ANALOG_CHANNELS];
        // % ratio of pre-trigger to post trigger samples
    	uint64_t capture_ratio;
        // total number of bytes of data sent for one sample across all channels
        uint16_t bytes_per_slice;
        //The number of bytes needed to store all channels for one sample in the device data buff
        uint32_t dig_sample_bytes;
/* Tracking/status once started */
        //number of bytes in the current serial input stream
        uint32_t bytes_avail; 
        //Samples sent to the session */
        uint32_t sent_samples;
       //count total received bytes to detect lost info*/
        uint64_t byte_cnt;
       //For SW based triggering we put the device into continuous transmit and stop when 
       // we detect a sample and capture all the samples we need. trigger_fired is thus set when
       // the sw trigger logic detects a trigger.
       //For non triggered modes we send a start and a number of samples and the device 
       //transmits that much. trigger_fired is set immediately at the start.
        gboolean trigger_fired;
        //Has the device, via an "!" indicated it has stopped sending data, or has a marker 
        //error been detected
  //        gboolean device_stopped;
       rxstate_t rxstate;
/* Serial Related */
	// Serial data buffer 
	unsigned char *buffer;
        //Size of incoming serial buffer
        uint32_t serial_buffer_size;
        //Current byte in serial read stream that is being processed
        uint32_t ser_rdptr;
       //write pointer into the serial input buffer
        uint32_t wrptr;

/* Buffering Related */
        /* parsed serial read data is split into each channels dedicated buffer for analog*/
        float  *a_data_bufs[MAX_ANALOG_CHANNELS];
        /*digital samples are stored packed together since cli/pulseview want it that way*/
        uint8_t *d_data_buf;
        /*write point for the the per channel data buffers*/
        uint32_t cbuf_wrptr; 
        /*size of packet data buffers for each channel*/
        uint32_t sample_buf_size;
/* RLE related*/
        /*Previous sample values to duplicate for rle */
        float a_last[MAX_ANALOG_CHANNELS];
        uint8_t d_last[MAX_DIGITAL_CHANNELS/8];

/* SW Trigger Related */
        struct soft_trigger_logic *stl;    
        //Maximum number of entries to store pre-trigger
         uint32_t pretrig_entries;  
        /* Analog pre-trigger storage for software based triggering
          because sw based only has internal storage for logic*/
        float  *a_pretrig_bufs[MAX_ANALOG_CHANNELS];
        uint32_t pretrig_wr_ptr;

};

SR_PRIV int raspberrypi_pico_receive(int fd, int revents, void *cb_data);
SR_PRIV int raspberrypi_pico_get_dev_cfg(const struct sr_dev_inst *sdi);

void process_D4(struct sr_dev_inst *sdi,struct dev_context *d);
void process_slice(struct sr_dev_inst *sdi,struct dev_context *devc);

int send_analog(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_samples, uint32_t offset);
int send_analog_ring(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_samples);

int process_group(struct sr_dev_inst *sdi,struct dev_context *devc,uint32_t num_slices);
void rle_memset(struct dev_context *devc,uint32_t num_slices);
SR_PRIV int check_marker(struct dev_context *d,int *len);



#endif
