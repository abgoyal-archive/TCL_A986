

#ifndef _HPI6205_H_
#define _HPI6205_H_

/* transitional conditional compile shared between host and DSP */
/* #define HPI6205_NO_HSR_POLL */

#include "hpi_internal.h"

#define H620_HIF_RESET          0
#define H620_HIF_IDLE           1
#define H620_HIF_GET_RESP       2
#define H620_HIF_DATA_DONE      3
#define H620_HIF_DATA_MASK      0x10
#define H620_HIF_SEND_DATA      0x14
#define H620_HIF_GET_DATA       0x15
#define H620_HIF_UNKNOWN                0x0000ffff


#define H620_MAX_ISTREAMS 32
#define H620_MAX_OSTREAMS 32
#define HPI_NMIXER_CONTROLS 2048

struct controlcache_6205 {
	u32 number_of_controls;
	u32 physical_address32;
	u32 size_in_bytes;
};

struct async_event_buffer_6205 {
	u32 physical_address32;
	u32 spare;
	struct hpi_fifo_buffer b;
};

#define HPI6205_SIZEOF_DATA (16*1024)
struct bus_master_interface {
	u32 host_cmd;
	u32 dsp_ack;
	u32 transfer_size_in_bytes;
	union {
		struct hpi_message message_buffer;
		struct hpi_response response_buffer;
		u8 b_data[HPI6205_SIZEOF_DATA];
	} u;
	struct controlcache_6205 control_cache;
	struct async_event_buffer_6205 async_buffer;
	struct hpi_hostbuffer_status
	 instream_host_buffer_status[H620_MAX_ISTREAMS];
	struct hpi_hostbuffer_status
	 outstream_host_buffer_status[H620_MAX_OSTREAMS];
};

#endif
