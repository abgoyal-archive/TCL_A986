
#ifndef _LINUX_HRTIMER_H
#define _LINUX_HRTIMER_H

#include <linux/rbtree.h>
#include <linux/ktime.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/percpu.h>
#include <linux/timer.h>


struct hrtimer_clock_base;
struct hrtimer_cpu_base;

enum hrtimer_mode {
	HRTIMER_MODE_ABS = 0x0,		/* Time value is absolute */
	HRTIMER_MODE_REL = 0x1,		/* Time value is relative to now */
	HRTIMER_MODE_PINNED = 0x02,	/* Timer is bound to CPU */
	HRTIMER_MODE_ABS_PINNED = 0x02,
	HRTIMER_MODE_REL_PINNED = 0x03,
};

enum hrtimer_restart {
	HRTIMER_NORESTART,	/* Timer is not restarted */
	HRTIMER_RESTART,	/* Timer must be restarted */
};

#define HRTIMER_STATE_INACTIVE	0x00
#define HRTIMER_STATE_ENQUEUED	0x01
#define HRTIMER_STATE_CALLBACK	0x02
#define HRTIMER_STATE_MIGRATE	0x04

struct hrtimer {
	struct rb_node			node;
	ktime_t				_expires;
	ktime_t				_softexpires;
	enum hrtimer_restart		(*function)(struct hrtimer *);
	struct hrtimer_clock_base	*base;
	unsigned long			state;
#ifdef CONFIG_TIMER_STATS
	int				start_pid;
	void				*start_site;
	char				start_comm[16];
#endif
};

struct hrtimer_sleeper {
	struct hrtimer timer;
	struct task_struct *task;
};

struct hrtimer_clock_base {
	struct hrtimer_cpu_base	*cpu_base;
	clockid_t		index;
	struct rb_root		active;
	struct rb_node		*first;
	ktime_t			resolution;
	ktime_t			(*get_time)(void);
	ktime_t			softirq_time;
#ifdef CONFIG_HIGH_RES_TIMERS
	ktime_t			offset;
#endif
};

#define HRTIMER_MAX_CLOCK_BASES 2

struct hrtimer_cpu_base {
	raw_spinlock_t			lock;
	struct hrtimer_clock_base	clock_base[HRTIMER_MAX_CLOCK_BASES];
#ifdef CONFIG_HIGH_RES_TIMERS
	ktime_t				expires_next;
	int				hres_active;
	int				hang_detected;
	unsigned long			nr_events;
	unsigned long			nr_retries;
	unsigned long			nr_hangs;
	ktime_t				max_hang_time;
#endif
};

static inline void hrtimer_set_expires(struct hrtimer *timer, ktime_t time)
{
	timer->_expires = time;
	timer->_softexpires = time;
}

static inline void hrtimer_set_expires_range(struct hrtimer *timer, ktime_t time, ktime_t delta)
{
	timer->_softexpires = time;
	timer->_expires = ktime_add_safe(time, delta);
}

static inline void hrtimer_set_expires_range_ns(struct hrtimer *timer, ktime_t time, unsigned long delta)
{
	timer->_softexpires = time;
	timer->_expires = ktime_add_safe(time, ns_to_ktime(delta));
}

static inline void hrtimer_set_expires_tv64(struct hrtimer *timer, s64 tv64)
{
	timer->_expires.tv64 = tv64;
	timer->_softexpires.tv64 = tv64;
}

static inline void hrtimer_add_expires(struct hrtimer *timer, ktime_t time)
{
	timer->_expires = ktime_add_safe(timer->_expires, time);
	timer->_softexpires = ktime_add_safe(timer->_softexpires, time);
}

static inline void hrtimer_add_expires_ns(struct hrtimer *timer, u64 ns)
{
	timer->_expires = ktime_add_ns(timer->_expires, ns);
	timer->_softexpires = ktime_add_ns(timer->_softexpires, ns);
}

static inline ktime_t hrtimer_get_expires(const struct hrtimer *timer)
{
	return timer->_expires;
}

static inline ktime_t hrtimer_get_softexpires(const struct hrtimer *timer)
{
	return timer->_softexpires;
}

static inline s64 hrtimer_get_expires_tv64(const struct hrtimer *timer)
{
	return timer->_expires.tv64;
}
static inline s64 hrtimer_get_softexpires_tv64(const struct hrtimer *timer)
{
	return timer->_softexpires.tv64;
}

static inline s64 hrtimer_get_expires_ns(const struct hrtimer *timer)
{
	return ktime_to_ns(timer->_expires);
}

static inline ktime_t hrtimer_expires_remaining(const struct hrtimer *timer)
{
    return ktime_sub(timer->_expires, timer->base->get_time());
}

#ifdef CONFIG_HIGH_RES_TIMERS
struct clock_event_device;

extern void clock_was_set(void);
extern void hres_timers_resume(void);
extern void hrtimer_interrupt(struct clock_event_device *dev);

static inline ktime_t hrtimer_cb_get_time(struct hrtimer *timer)
{
	return timer->base->get_time();
}

static inline int hrtimer_is_hres_active(struct hrtimer *timer)
{
	return timer->base->cpu_base->hres_active;
}

extern void hrtimer_peek_ahead_timers(void);

# define HIGH_RES_NSEC		1
# define KTIME_HIGH_RES		(ktime_t) { .tv64 = HIGH_RES_NSEC }
# define MONOTONIC_RES_NSEC	HIGH_RES_NSEC
# define KTIME_MONOTONIC_RES	KTIME_HIGH_RES

#else

# define MONOTONIC_RES_NSEC	LOW_RES_NSEC
# define KTIME_MONOTONIC_RES	KTIME_LOW_RES

static inline void clock_was_set(void) { }
static inline void hrtimer_peek_ahead_timers(void) { }

static inline void hres_timers_resume(void) { }

static inline ktime_t hrtimer_cb_get_time(struct hrtimer *timer)
{
	return timer->base->softirq_time;
}

static inline int hrtimer_is_hres_active(struct hrtimer *timer)
{
	return 0;
}
#endif

extern ktime_t ktime_get(void);
extern ktime_t ktime_get_real(void);


DECLARE_PER_CPU(struct tick_device, tick_cpu_device);


/* Exported timer functions: */

/* Initialize timers: */
extern void hrtimer_init(struct hrtimer *timer, clockid_t which_clock,
			 enum hrtimer_mode mode);

#ifdef CONFIG_DEBUG_OBJECTS_TIMERS
extern void hrtimer_init_on_stack(struct hrtimer *timer, clockid_t which_clock,
				  enum hrtimer_mode mode);

extern void destroy_hrtimer_on_stack(struct hrtimer *timer);
#else
static inline void hrtimer_init_on_stack(struct hrtimer *timer,
					 clockid_t which_clock,
					 enum hrtimer_mode mode)
{
	hrtimer_init(timer, which_clock, mode);
}
static inline void destroy_hrtimer_on_stack(struct hrtimer *timer) { }
#endif

/* Basic timer operations: */
extern int hrtimer_start(struct hrtimer *timer, ktime_t tim,
			 const enum hrtimer_mode mode);
extern int hrtimer_start_range_ns(struct hrtimer *timer, ktime_t tim,
			unsigned long range_ns, const enum hrtimer_mode mode);
extern int
__hrtimer_start_range_ns(struct hrtimer *timer, ktime_t tim,
			 unsigned long delta_ns,
			 const enum hrtimer_mode mode, int wakeup);

extern int hrtimer_cancel(struct hrtimer *timer);
extern int hrtimer_try_to_cancel(struct hrtimer *timer);

static inline int hrtimer_start_expires(struct hrtimer *timer,
						enum hrtimer_mode mode)
{
	unsigned long delta;
	ktime_t soft, hard;
	soft = hrtimer_get_softexpires(timer);
	hard = hrtimer_get_expires(timer);
	delta = ktime_to_ns(ktime_sub(hard, soft));
	return hrtimer_start_range_ns(timer, soft, delta, mode);
}

static inline int hrtimer_restart(struct hrtimer *timer)
{
	return hrtimer_start_expires(timer, HRTIMER_MODE_ABS);
}

/* Query timers: */
extern ktime_t hrtimer_get_remaining(const struct hrtimer *timer);
extern int hrtimer_get_res(const clockid_t which_clock, struct timespec *tp);

extern ktime_t hrtimer_get_next_event(void);

static inline int hrtimer_active(const struct hrtimer *timer)
{
	return timer->state != HRTIMER_STATE_INACTIVE;
}

static inline int hrtimer_is_queued(struct hrtimer *timer)
{
	return timer->state & HRTIMER_STATE_ENQUEUED;
}

static inline int hrtimer_callback_running(struct hrtimer *timer)
{
	return timer->state & HRTIMER_STATE_CALLBACK;
}

/* Forward a hrtimer so it expires after now: */
extern u64
hrtimer_forward(struct hrtimer *timer, ktime_t now, ktime_t interval);

/* Forward a hrtimer so it expires after the hrtimer's current now */
static inline u64 hrtimer_forward_now(struct hrtimer *timer,
				      ktime_t interval)
{
	return hrtimer_forward(timer, timer->base->get_time(), interval);
}

/* Precise sleep: */
extern long hrtimer_nanosleep(struct timespec *rqtp,
			      struct timespec __user *rmtp,
			      const enum hrtimer_mode mode,
			      const clockid_t clockid);
extern long hrtimer_nanosleep_restart(struct restart_block *restart_block);

extern void hrtimer_init_sleeper(struct hrtimer_sleeper *sl,
				 struct task_struct *tsk);

extern int schedule_hrtimeout_range(ktime_t *expires, unsigned long delta,
						const enum hrtimer_mode mode);
extern int schedule_hrtimeout_range_clock(ktime_t *expires,
		unsigned long delta, const enum hrtimer_mode mode, int clock);
extern int schedule_hrtimeout(ktime_t *expires, const enum hrtimer_mode mode);

/* Soft interrupt function to run the hrtimer queues: */
extern void hrtimer_run_queues(void);
extern void hrtimer_run_pending(void);

/* Bootup initialization: */
extern void __init hrtimers_init(void);

#if BITS_PER_LONG < 64
extern u64 ktime_divns(const ktime_t kt, s64 div);
#else /* BITS_PER_LONG < 64 */
# define ktime_divns(kt, div)		(u64)((kt).tv64 / (div))
#endif

/* Show pending timers: */
extern void sysrq_timer_list_show(void);

#endif
