

#include <linux/kernel.h>
#include <linux/slab.h>

struct fiq_debugger_ringbuf {
	int len;
	int head;
	int tail;
	u8 buf[];
};


static inline struct fiq_debugger_ringbuf *fiq_debugger_ringbuf_alloc(int len)
{
	struct fiq_debugger_ringbuf *rbuf;

	rbuf = kzalloc(sizeof(*rbuf) + len, GFP_KERNEL);
	if (rbuf == NULL)
		return NULL;

	rbuf->len = len;
	rbuf->head = 0;
	rbuf->tail = 0;
	smp_mb();

	return rbuf;
}

static inline void fiq_debugger_ringbuf_free(struct fiq_debugger_ringbuf *rbuf)
{
	kfree(rbuf);
}

static inline int fiq_debugger_ringbuf_level(struct fiq_debugger_ringbuf *rbuf)
{
	int level = rbuf->head - rbuf->tail;

	if (level < 0)
		level = rbuf->len + level;

	return level;
}

static inline int fiq_debugger_ringbuf_room(struct fiq_debugger_ringbuf *rbuf)
{
	return rbuf->len - fiq_debugger_ringbuf_level(rbuf) - 1;
}

static inline u8
fiq_debugger_ringbuf_peek(struct fiq_debugger_ringbuf *rbuf, int i)
{
	return rbuf->buf[(rbuf->tail + i) % rbuf->len];
}

static inline int
fiq_debugger_ringbuf_consume(struct fiq_debugger_ringbuf *rbuf, int count)
{
	count = min(count, fiq_debugger_ringbuf_level(rbuf));

	rbuf->tail = (rbuf->tail + count) % rbuf->len;
	smp_mb();

	return count;
}

static inline int
fiq_debugger_ringbuf_push(struct fiq_debugger_ringbuf *rbuf, u8 datum)
{
	if (fiq_debugger_ringbuf_room(rbuf) == 0)
		return 0;

	rbuf->buf[rbuf->head] = datum;
	smp_mb();
	rbuf->head = (rbuf->head + 1) % rbuf->len;
	smp_mb();

	return 1;
}
