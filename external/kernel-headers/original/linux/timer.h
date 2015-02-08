#ifndef _LINUX_TIMER_H
#define _LINUX_TIMER_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/stddef.h>

struct tvec_t_base_s;

struct timer_list {
	struct list_head entry;
	unsigned long expires;

	void (*function)(unsigned long);
	unsigned long data;

	struct tvec_t_base_s *base;
};

extern struct tvec_t_base_s boot_tvec_bases;

#define TIMER_INITIALIZER(_function, _expires, _data) {		\
		.function = (_function),			\
		.expires = (_expires),				\
		.data = (_data),				\
		.base = &boot_tvec_bases,			\
	}

#define DEFINE_TIMER(_name, _function, _expires, _data)		\
	struct timer_list _name =				\
		TIMER_INITIALIZER(_function, _expires, _data)

void fastcall init_timer(struct timer_list * timer);

static inline void setup_timer(struct timer_list * timer,
				void (*function)(unsigned long),
				unsigned long data)
{
	timer->function = function;
	timer->data = data;
	init_timer(timer);
}

static inline int timer_pending(const struct timer_list * timer)
{
	return timer->entry.next != NULL;
}

extern void add_timer_on(struct timer_list *timer, int cpu);
extern int del_timer(struct timer_list * timer);
extern int __mod_timer(struct timer_list *timer, unsigned long expires);
extern int mod_timer(struct timer_list *timer, unsigned long expires);

extern unsigned long next_timer_interrupt(void);

static inline void add_timer(struct timer_list *timer)
{
	BUG_ON(timer_pending(timer));
	__mod_timer(timer, timer->expires);
}

#ifdef CONFIG_SMP
  extern int try_to_del_timer_sync(struct timer_list *timer);
  extern int del_timer_sync(struct timer_list *timer);
#else
# define try_to_del_timer_sync(t)	del_timer(t)
# define del_timer_sync(t)		del_timer(t)
#endif

#define del_singleshot_timer_sync(t) del_timer_sync(t)

extern void init_timers(void);
extern void run_local_timers(void);
struct hrtimer;
extern int it_real_fn(struct hrtimer *);

#endif
