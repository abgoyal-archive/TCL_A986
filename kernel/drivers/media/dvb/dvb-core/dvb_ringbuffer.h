

#ifndef _DVB_RINGBUFFER_H_
#define _DVB_RINGBUFFER_H_

#include <linux/spinlock.h>
#include <linux/wait.h>

struct dvb_ringbuffer {
	u8               *data;
	ssize_t           size;
	ssize_t           pread;
	ssize_t           pwrite;
	int               error;

	wait_queue_head_t queue;
	spinlock_t        lock;
};

#define DVB_RINGBUFFER_PKTHDRSIZE 3



/* initialize ring buffer, lock and queue */
extern void dvb_ringbuffer_init(struct dvb_ringbuffer *rbuf, void *data, size_t len);

/* test whether buffer is empty */
extern int dvb_ringbuffer_empty(struct dvb_ringbuffer *rbuf);

/* return the number of free bytes in the buffer */
extern ssize_t dvb_ringbuffer_free(struct dvb_ringbuffer *rbuf);

/* return the number of bytes waiting in the buffer */
extern ssize_t dvb_ringbuffer_avail(struct dvb_ringbuffer *rbuf);


extern void dvb_ringbuffer_reset(struct dvb_ringbuffer *rbuf);


/* read routines & macros */
/* ---------------------- */
/* flush buffer */
extern void dvb_ringbuffer_flush(struct dvb_ringbuffer *rbuf);

/* flush buffer protected by spinlock and wake-up waiting task(s) */
extern void dvb_ringbuffer_flush_spinlock_wakeup(struct dvb_ringbuffer *rbuf);

/* peek at byte <offs> in the buffer */
#define DVB_RINGBUFFER_PEEK(rbuf,offs)	\
			(rbuf)->data[((rbuf)->pread+(offs))%(rbuf)->size]

/* advance read ptr by <num> bytes */
#define DVB_RINGBUFFER_SKIP(rbuf,num)	\
			(rbuf)->pread=((rbuf)->pread+(num))%(rbuf)->size

extern ssize_t dvb_ringbuffer_read_user(struct dvb_ringbuffer *rbuf,
				   u8 __user *buf, size_t len);
extern void dvb_ringbuffer_read(struct dvb_ringbuffer *rbuf,
				   u8 *buf, size_t len);


/* write routines & macros */
/* ----------------------- */
/* write single byte to ring buffer */
#define DVB_RINGBUFFER_WRITE_BYTE(rbuf,byte)	\
			{ (rbuf)->data[(rbuf)->pwrite]=(byte); \
			(rbuf)->pwrite=((rbuf)->pwrite+1)%(rbuf)->size; }
extern ssize_t dvb_ringbuffer_write(struct dvb_ringbuffer *rbuf, const u8 *buf,
				    size_t len);


extern ssize_t dvb_ringbuffer_pkt_write(struct dvb_ringbuffer *rbuf, u8* buf,
					size_t len);

extern ssize_t dvb_ringbuffer_pkt_read_user(struct dvb_ringbuffer *rbuf, size_t idx,
				       int offset, u8 __user *buf, size_t len);
extern ssize_t dvb_ringbuffer_pkt_read(struct dvb_ringbuffer *rbuf, size_t idx,
				       int offset, u8 *buf, size_t len);

extern void dvb_ringbuffer_pkt_dispose(struct dvb_ringbuffer *rbuf, size_t idx);

extern ssize_t dvb_ringbuffer_pkt_next(struct dvb_ringbuffer *rbuf, size_t idx, size_t* pktlen);


#endif /* _DVB_RINGBUFFER_H_ */
