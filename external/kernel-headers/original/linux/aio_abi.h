#ifndef __LINUX__AIO_ABI_H
#define __LINUX__AIO_ABI_H

#include <asm/byteorder.h>

typedef unsigned long	aio_context_t;

enum {
	IOCB_CMD_PREAD = 0,
	IOCB_CMD_PWRITE = 1,
	IOCB_CMD_FSYNC = 2,
	IOCB_CMD_FDSYNC = 3,
	/* These two are experimental.
	 * IOCB_CMD_PREADX = 4,
	 * IOCB_CMD_POLL = 5,
	 */
	IOCB_CMD_NOOP = 6,
};

/* read() from /dev/aio returns these structures. */
struct io_event {
	__u64		data;		/* the data field from the iocb */
	__u64		obj;		/* what iocb this event came from */
	__s64		res;		/* result code for this event */
	__s64		res2;		/* secondary result */
};

#if defined(__LITTLE_ENDIAN)
#define PADDED(x,y)	x, y
#elif defined(__BIG_ENDIAN)
#define PADDED(x,y)	y, x
#else
#error edit for your odd byteorder.
#endif


struct iocb {
	/* these are internal to the kernel/libc. */
	__u64	aio_data;	/* data to be returned in event's data */
	__u32	PADDED(aio_key, aio_reserved1);
				/* the kernel sets aio_key to the req # */

	/* common fields */
	__u16	aio_lio_opcode;	/* see IOCB_CMD_ above */
	__s16	aio_reqprio;
	__u32	aio_fildes;

	__u64	aio_buf;
	__u64	aio_nbytes;
	__s64	aio_offset;

	/* extra parameters */
	__u64	aio_reserved2;	/* TODO: use this for a (struct sigevent *) */
	__u64	aio_reserved3;
}; /* 64 bytes */

#undef IFBIG
#undef IFLITTLE

#endif /* __LINUX__AIO_ABI_H */

