

#ifndef TIMER_H
#define TIMER_H

#include <linux/timer.h>
#include <linux/jiffies.h>

#include <asm/param.h>  /* for HZ */

#include <net/irda/irda.h>

/* A few forward declarations (to make compiler happy) */
struct irlmp_cb;
struct irlap_cb;
struct lsap_cb;
struct lap_cb;

#define POLL_TIMEOUT        (450*HZ/1000)    /* Must never exceed 500 ms */
#define FINAL_TIMEOUT       (500*HZ/1000)    /* Must never exceed 500 ms */

#define WD_TIMEOUT          (POLL_TIMEOUT*2)

#define MEDIABUSY_TIMEOUT   (500*HZ/1000)    /* 500 msec */
#define SMALLBUSY_TIMEOUT   (100*HZ/1000)    /* 100 msec - IrLAP 6.13.4 */

#define SLOT_TIMEOUT            (90*HZ/1000)

#define XIDEXTRA_TIMEOUT        (34*HZ/1000)  /* 34 msec */

#define WATCHDOG_TIMEOUT        (20*HZ)       /* 20 sec */

typedef void (*TIMER_CALLBACK)(void *);

static inline void irda_start_timer(struct timer_list *ptimer, int timeout, 
				    void* data, TIMER_CALLBACK callback)
{
	ptimer->function = (void (*)(unsigned long)) callback;
	ptimer->data = (unsigned long) data;
	
	/* Set new value for timer (update or add timer).
	 * We use mod_timer() because it's more efficient and also
	 * safer with respect to race conditions - Jean II */
	mod_timer(ptimer, jiffies + timeout);
}


void irlap_start_slot_timer(struct irlap_cb *self, int timeout);
void irlap_start_query_timer(struct irlap_cb *self, int S, int s);
void irlap_start_final_timer(struct irlap_cb *self, int timeout);
void irlap_start_wd_timer(struct irlap_cb *self, int timeout);
void irlap_start_backoff_timer(struct irlap_cb *self, int timeout);

void irlap_start_mbusy_timer(struct irlap_cb *self, int timeout);
void irlap_stop_mbusy_timer(struct irlap_cb *);

void irlmp_start_watchdog_timer(struct lsap_cb *, int timeout);
void irlmp_start_discovery_timer(struct irlmp_cb *, int timeout);
void irlmp_start_idle_timer(struct lap_cb *, int timeout);
void irlmp_stop_idle_timer(struct lap_cb *self);

#endif

