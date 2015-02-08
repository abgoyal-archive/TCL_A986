


#ifndef _LINUX_KFIFO_H
#define _LINUX_KFIFO_H

#include <linux/kernel.h>
#include <linux/spinlock.h>

struct kfifo {
	unsigned char *buffer;	/* the buffer holding the data */
	unsigned int size;	/* the size of the allocated buffer */
	unsigned int in;	/* data is added at offset (in % size) */
	unsigned int out;	/* data is extracted from off. (out % size) */
};


/* helper macro */
#define __kfifo_initializer(s, b) \
	(struct kfifo) { \
		.size	= s, \
		.in	= 0, \
		.out	= 0, \
		.buffer = b \
	}

#define DECLARE_KFIFO(name, size) \
union { \
	struct kfifo name; \
	unsigned char name##kfifo_buffer[size + sizeof(struct kfifo)]; \
}

#define INIT_KFIFO(name) \
	name = __kfifo_initializer(sizeof(name##kfifo_buffer) - \
				sizeof(struct kfifo), \
				name##kfifo_buffer + sizeof(struct kfifo))

#define DEFINE_KFIFO(name, size) \
	unsigned char name##kfifo_buffer[size]; \
	struct kfifo name = __kfifo_initializer(size, name##kfifo_buffer)

extern void kfifo_init(struct kfifo *fifo, void *buffer,
			unsigned int size);
extern __must_check int kfifo_alloc(struct kfifo *fifo, unsigned int size,
			gfp_t gfp_mask);
extern void kfifo_free(struct kfifo *fifo);
extern unsigned int kfifo_in(struct kfifo *fifo,
				const void *from, unsigned int len);
extern __must_check unsigned int kfifo_out(struct kfifo *fifo,
				void *to, unsigned int len);
extern __must_check unsigned int kfifo_out_peek(struct kfifo *fifo,
				void *to, unsigned int len, unsigned offset);

static inline bool kfifo_initialized(struct kfifo *fifo)
{
	return fifo->buffer != NULL;
}

static inline void kfifo_reset(struct kfifo *fifo)
{
	fifo->in = fifo->out = 0;
}

static inline void kfifo_reset_out(struct kfifo *fifo)
{
	smp_mb();
	fifo->out = fifo->in;
}

static inline __must_check unsigned int kfifo_size(struct kfifo *fifo)
{
	return fifo->size;
}

static inline unsigned int kfifo_len(struct kfifo *fifo)
{
	register unsigned int	out;

	out = fifo->out;
	smp_rmb();
	return fifo->in - out;
}

static inline __must_check int kfifo_is_empty(struct kfifo *fifo)
{
	return fifo->in == fifo->out;
}

static inline __must_check int kfifo_is_full(struct kfifo *fifo)
{
	return kfifo_len(fifo) == kfifo_size(fifo);
}

static inline __must_check unsigned int kfifo_avail(struct kfifo *fifo)
{
	return kfifo_size(fifo) - kfifo_len(fifo);
}

static inline unsigned int kfifo_in_locked(struct kfifo *fifo,
		const void *from, unsigned int n, spinlock_t *lock)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(lock, flags);

	ret = kfifo_in(fifo, from, n);

	spin_unlock_irqrestore(lock, flags);

	return ret;
}

static inline __must_check unsigned int kfifo_out_locked(struct kfifo *fifo,
	void *to, unsigned int n, spinlock_t *lock)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(lock, flags);

	ret = kfifo_out(fifo, to, n);

	spin_unlock_irqrestore(lock, flags);

	return ret;
}

extern void kfifo_skip(struct kfifo *fifo, unsigned int len);

extern __must_check int kfifo_from_user(struct kfifo *fifo,
	const void __user *from, unsigned int n, unsigned *lenout);

extern __must_check int kfifo_to_user(struct kfifo *fifo,
	void __user *to, unsigned int n, unsigned *lenout);

static inline void __kfifo_add_out(struct kfifo *fifo,
				unsigned int off)
{
	smp_mb();
	fifo->out += off;
}

static inline void __kfifo_add_in(struct kfifo *fifo,
				unsigned int off)
{
	smp_wmb();
	fifo->in += off;
}

static inline unsigned int __kfifo_off(struct kfifo *fifo, unsigned int off)
{
	return off & (fifo->size - 1);
}

static inline unsigned int __kfifo_peek_n(struct kfifo *fifo,
				unsigned int recsize)
{
#define __KFIFO_GET(fifo, off, shift) \
	((fifo)->buffer[__kfifo_off((fifo), (fifo)->out+(off))] << (shift))

	unsigned int l;

	l = __KFIFO_GET(fifo, 0, 0);

	if (--recsize)
		l |= __KFIFO_GET(fifo, 1, 8);

	return l;
#undef	__KFIFO_GET
}

static inline void __kfifo_poke_n(struct kfifo *fifo,
			unsigned int recsize, unsigned int n)
{
#define __KFIFO_PUT(fifo, off, val, shift) \
		( \
		(fifo)->buffer[__kfifo_off((fifo), (fifo)->in+(off))] = \
		(unsigned char)((val) >> (shift)) \
		)

	__KFIFO_PUT(fifo, 0, n, 0);

	if (--recsize)
		__KFIFO_PUT(fifo, 1, n, 8);
#undef	__KFIFO_PUT
}

extern unsigned int __kfifo_in_n(struct kfifo *fifo,
	const void *from, unsigned int n, unsigned int recsize);

extern unsigned int __kfifo_in_generic(struct kfifo *fifo,
	const void *from, unsigned int n, unsigned int recsize);

static inline unsigned int __kfifo_in_rec(struct kfifo *fifo,
	const void *from, unsigned int n, unsigned int recsize)
{
	unsigned int ret;

	ret = __kfifo_in_n(fifo, from, n, recsize);

	if (likely(ret == 0)) {
		if (recsize)
			__kfifo_poke_n(fifo, recsize, n);
		__kfifo_add_in(fifo, n + recsize);
	}
	return ret;
}

static inline __must_check unsigned int kfifo_in_rec(struct kfifo *fifo,
	void *from, unsigned int n, unsigned int recsize)
{
	if (!__builtin_constant_p(recsize))
		return __kfifo_in_generic(fifo, from, n, recsize);
	return __kfifo_in_rec(fifo, from, n, recsize);
}

extern unsigned int __kfifo_out_n(struct kfifo *fifo,
	void *to, unsigned int reclen, unsigned int recsize);

extern unsigned int __kfifo_out_generic(struct kfifo *fifo,
	void *to, unsigned int n,
	unsigned int recsize, unsigned int *total);

static inline unsigned int __kfifo_out_rec(struct kfifo *fifo,
	void *to, unsigned int n, unsigned int recsize,
	unsigned int *total)
{
	unsigned int l;

	if (!recsize) {
		l = n;
		if (total)
			*total = l;
	} else {
		l = __kfifo_peek_n(fifo, recsize);
		if (total)
			*total = l;
		if (n < l)
			return l;
	}

	return __kfifo_out_n(fifo, to, l, recsize);
}

static inline __must_check unsigned int kfifo_out_rec(struct kfifo *fifo,
	void *to, unsigned int n, unsigned int recsize,
	unsigned int *total)

{
	if (!__builtin_constant_p(recsize))
		return __kfifo_out_generic(fifo, to, n, recsize, total);
	return __kfifo_out_rec(fifo, to, n, recsize, total);
}

extern unsigned int __kfifo_from_user_n(struct kfifo *fifo,
	const void __user *from, unsigned int n, unsigned int recsize);

extern unsigned int __kfifo_from_user_generic(struct kfifo *fifo,
	const void __user *from, unsigned int n, unsigned int recsize);

static inline unsigned int __kfifo_from_user_rec(struct kfifo *fifo,
	const void __user *from, unsigned int n, unsigned int recsize)
{
	unsigned int ret;

	ret = __kfifo_from_user_n(fifo, from, n, recsize);

	if (likely(ret == 0)) {
		if (recsize)
			__kfifo_poke_n(fifo, recsize, n);
		__kfifo_add_in(fifo, n + recsize);
	}
	return ret;
}

static inline __must_check unsigned int kfifo_from_user_rec(struct kfifo *fifo,
	const void __user *from, unsigned int n, unsigned int recsize)
{
	if (!__builtin_constant_p(recsize))
		return __kfifo_from_user_generic(fifo, from, n, recsize);
	return __kfifo_from_user_rec(fifo, from, n, recsize);
}

extern unsigned int __kfifo_to_user_n(struct kfifo *fifo,
	void __user *to, unsigned int n, unsigned int reclen,
	unsigned int recsize);

extern unsigned int __kfifo_to_user_generic(struct kfifo *fifo,
	void __user *to, unsigned int n, unsigned int recsize,
	unsigned int *total);

static inline unsigned int __kfifo_to_user_rec(struct kfifo *fifo,
	void __user *to, unsigned int n,
	unsigned int recsize, unsigned int *total)
{
	unsigned int l;

	if (!recsize) {
		l = n;
		if (total)
			*total = l;
	} else {
		l = __kfifo_peek_n(fifo, recsize);
		if (total)
			*total = l;
		if (n < l)
			return l;
	}

	return __kfifo_to_user_n(fifo, to, n, l, recsize);
}

static inline __must_check unsigned int kfifo_to_user_rec(struct kfifo *fifo,
		void __user *to, unsigned int n, unsigned int recsize,
		unsigned int *total)
{
	if (!__builtin_constant_p(recsize))
		return __kfifo_to_user_generic(fifo, to, n, recsize, total);
	return __kfifo_to_user_rec(fifo, to, n, recsize, total);
}

extern unsigned int __kfifo_peek_generic(struct kfifo *fifo,
				unsigned int recsize);

static inline __must_check unsigned int kfifo_peek_rec(struct kfifo *fifo,
	unsigned int recsize)
{
	if (!__builtin_constant_p(recsize))
		return __kfifo_peek_generic(fifo, recsize);
	if (!recsize)
		return kfifo_len(fifo);
	return __kfifo_peek_n(fifo, recsize);
}

extern void __kfifo_skip_generic(struct kfifo *fifo, unsigned int recsize);

static inline void __kfifo_skip_rec(struct kfifo *fifo,
	unsigned int recsize)
{
	unsigned int l;

	if (recsize) {
		l = __kfifo_peek_n(fifo, recsize);

		if (l + recsize <= kfifo_len(fifo)) {
			__kfifo_add_out(fifo, l + recsize);
			return;
		}
	}
	kfifo_reset_out(fifo);
}

static inline void kfifo_skip_rec(struct kfifo *fifo,
	unsigned int recsize)
{
	if (!__builtin_constant_p(recsize))
		__kfifo_skip_generic(fifo, recsize);
	else
		__kfifo_skip_rec(fifo, recsize);
}

static inline __must_check unsigned int kfifo_avail_rec(struct kfifo *fifo,
	unsigned int recsize)
{
	unsigned int l = kfifo_size(fifo) - kfifo_len(fifo);

	return (l > recsize) ? l - recsize : 0;
}

#endif
