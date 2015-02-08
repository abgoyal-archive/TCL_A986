#ifndef __LINUX_SEQLOCK_H
#define __LINUX_SEQLOCK_H

#include <linux/spinlock.h>
#include <linux/preempt.h>

typedef struct {
	unsigned sequence;
	spinlock_t lock;
} seqlock_t;

#define __SEQLOCK_UNLOCKED(lockname) \
		 { 0, __SPIN_LOCK_UNLOCKED(lockname) }

#define SEQLOCK_UNLOCKED \
		 __SEQLOCK_UNLOCKED(old_style_seqlock_init)

#define seqlock_init(x) \
		do { *(x) = (seqlock_t) __SEQLOCK_UNLOCKED(x); } while (0)

#define DEFINE_SEQLOCK(x) \
		seqlock_t x = __SEQLOCK_UNLOCKED(x)

static inline void write_seqlock(seqlock_t *sl)
{
	spin_lock(&sl->lock);
	++sl->sequence;
	smp_wmb();			
}	

static inline void write_sequnlock(seqlock_t *sl) 
{
	smp_wmb();
	sl->sequence++;
	spin_unlock(&sl->lock);
}

static inline int write_tryseqlock(seqlock_t *sl)
{
	int ret = spin_trylock(&sl->lock);

	if (ret) {
		++sl->sequence;
		smp_wmb();			
	}
	return ret;
}

/* Start of read calculation -- fetch last complete writer token */
static __always_inline unsigned read_seqbegin(const seqlock_t *sl)
{
	unsigned ret = sl->sequence;
	smp_rmb();
	return ret;
}

static __always_inline int read_seqretry(const seqlock_t *sl, unsigned iv)
{
	smp_rmb();
	return (iv & 1) | (sl->sequence ^ iv);
}



typedef struct seqcount {
	unsigned sequence;
} seqcount_t;

#define SEQCNT_ZERO { 0 }
#define seqcount_init(x)	do { *(x) = (seqcount_t) SEQCNT_ZERO; } while (0)

/* Start of read using pointer to a sequence counter only.  */
static inline unsigned read_seqcount_begin(const seqcount_t *s)
{
	unsigned ret = s->sequence;
	smp_rmb();
	return ret;
}

static inline int read_seqcount_retry(const seqcount_t *s, unsigned iv)
{
	smp_rmb();
	return (iv & 1) | (s->sequence ^ iv);
}


static inline void write_seqcount_begin(seqcount_t *s)
{
	s->sequence++;
	smp_wmb();
}

static inline void write_seqcount_end(seqcount_t *s)
{
	smp_wmb();
	s->sequence++;
}

#define write_seqlock_irqsave(lock, flags)				\
	do { local_irq_save(flags); write_seqlock(lock); } while (0)
#define write_seqlock_irq(lock)						\
	do { local_irq_disable();   write_seqlock(lock); } while (0)
#define write_seqlock_bh(lock)						\
        do { local_bh_disable();    write_seqlock(lock); } while (0)

#define write_sequnlock_irqrestore(lock, flags)				\
	do { write_sequnlock(lock); local_irq_restore(flags); } while(0)
#define write_sequnlock_irq(lock)					\
	do { write_sequnlock(lock); local_irq_enable(); } while(0)
#define write_sequnlock_bh(lock)					\
	do { write_sequnlock(lock); local_bh_enable(); } while(0)

#define read_seqbegin_irqsave(lock, flags)				\
	({ local_irq_save(flags);   read_seqbegin(lock); })

#define read_seqretry_irqrestore(lock, iv, flags)			\
	({								\
		int ret = read_seqretry(lock, iv);			\
		local_irq_restore(flags);				\
		ret;							\
	})

#endif /* __LINUX_SEQLOCK_H */
