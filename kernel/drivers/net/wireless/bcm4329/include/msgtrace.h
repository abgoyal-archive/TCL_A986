

#ifndef	_MSGTRACE_H
#define	_MSGTRACE_H

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif


/* This marks the start of a packed structure section. */
#include <packed_section_start.h>

#define MSGTRACE_VERSION 1

/* Message trace header */
typedef BWL_PRE_PACKED_STRUCT struct msgtrace_hdr {
	uint8	version;
	uint8   spare;
	uint16	len;	/* Len of the trace */
	uint32	seqnum;	/* Sequence number of message. Useful if the messsage has been lost
			 * because of DMA error or a bus reset (ex: SDIO Func2)
			 */
	uint32  discarded_bytes;  /* Number of discarded bytes because of trace overflow  */
	uint32  discarded_printf; /* Number of discarded printf because of trace overflow */
} BWL_POST_PACKED_STRUCT msgtrace_hdr_t;

#define MSGTRACE_HDRLEN 	sizeof(msgtrace_hdr_t)

extern bool msgtrace_hbus_trace;

typedef void (*msgtrace_func_send_t)(void *hdl1, void *hdl2, uint8 *hdr,
                                     uint16 hdrlen, uint8 *buf, uint16 buflen);

extern void msgtrace_sent(void);
extern void msgtrace_put(char *buf, int count);
extern void msgtrace_init(void *hdl1, void *hdl2, msgtrace_func_send_t func_send);

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif	/* _MSGTRACE_H */
