

#ifndef _LINUX_ANDROID_ALARM_H
#define _LINUX_ANDROID_ALARM_H

#include <linux/ioctl.h>
#include <linux/time.h>
#include <linux/rtc.h>

enum android_alarm_type {
	/* return code bit numbers or set alarm arg */
	ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_SYSTEMTIME,

	ANDROID_ALARM_TYPE_COUNT,

	ANDROID_ALARM_POWER_ON = 6,

	/* return code bit numbers */
	/* ANDROID_ALARM_TIME_CHANGE = 16 */
};

#ifdef __KERNEL__

#include <linux/ktime.h>
#include <linux/rbtree.h>
#include <linux/rtc.h>



struct alarm {
	struct rb_node 		node;
	enum android_alarm_type type;
	ktime_t			softexpires;
	ktime_t			expires;
	void			(*function)(struct alarm *);
};

void alarm_init(struct alarm *alarm,
	enum android_alarm_type type, void (*function)(struct alarm *));
void alarm_start_range(struct alarm *alarm, ktime_t start, ktime_t end);
int alarm_try_to_cancel(struct alarm *alarm);
int alarm_cancel(struct alarm *alarm);
ktime_t alarm_get_elapsed_realtime(void);

/* set rtc while preserving elapsed realtime */
int alarm_set_rtc(struct timespec new_time, struct rtc_time *prev_time);

void alarm_set_power_on(struct timespec new_alarm_time);
void alarm_get_power_on(struct rtc_wkalrm *alm);

#endif

enum android_alarm_return_flags {
	ANDROID_ALARM_RTC_WAKEUP_MASK = 1U << ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC_MASK = 1U << ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP_MASK =
				1U << ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME_MASK =
				1U << ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_SYSTEMTIME_MASK = 1U << ANDROID_ALARM_SYSTEMTIME,
	ANDROID_ALARM_TIME_CHANGE_MASK = 1U << 16
};

/* Disable alarm */
#define ANDROID_ALARM_CLEAR(type)           _IO('a', 0 | ((type) << 4))

/* Ack last alarm and wait for next */
#define ANDROID_ALARM_WAIT                  _IO('a', 1)

#define ALARM_IOW(c, type, size)            _IOW('a', (c) | ((type) << 4), size)
/* Set alarm */
#define ANDROID_ALARM_SET(type)             ALARM_IOW(2, type, struct timespec)
#define ANDROID_ALARM_SET_AND_WAIT(type)    ALARM_IOW(3, type, struct timespec)
#define ANDROID_ALARM_GET_TIME(type)        ALARM_IOW(4, type, struct timespec)
#define ANDROID_ALARM_SET_RTC               _IOW('a', 5, struct timespec)
//#define ANDROID_ALARM_SET_TIMEZONE        _IOW('a', 6, struct timezone)
#define ANDROID_ALARM_GET_POWER_ON          _IOR('a', 7, struct rtc_wkalrm)
#define ANDROID_ALARM_BASE_CMD(cmd)         (cmd & ~(_IOC(0, 0, 0xf0, 0)))
#define ANDROID_ALARM_IOCTL_TO_TYPE(cmd)    (_IOC_NR(cmd) >> 4)

#endif
