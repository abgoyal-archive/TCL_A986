

#ifndef _MT6573_KPD_H_
#define _MT6573_KPD_H_

#include <linux/kernel.h>
#include <cust_kpd.h>

#define KPD_DRV_CTRL_BACKLIGHT	KPD_NO	/* retired, move to Lights framework */
#define KPD_BACKLIGHT_TIME	8	/* sec */
/* the keys can wake up the system and we should enable backlight */
#define KPD_BACKLIGHT_WAKE_KEY	\
{				\
	KEY_ENDCALL, KEY_POWER,	\
}

#define KPD_HAS_SLIDE_QWERTY	KPD_NO
#define KPD_SLIDE_EINT		CUST_EINT_KPD_SLIDE_NUM
#define KPD_SLIDE_DEBOUNCE	CUST_EINT_KPD_SLIDE_DEBOUNCE_CN	/* (cn / 32) ms */
#define KPD_SLIDE_POLARITY	CUST_EINT_KPD_SLIDE_POLARITY
#define KPD_SLIDE_SENSITIVE	CUST_EINT_KPD_SLIDE_SENSITIVE

#if KPD_DRV_CTRL_BACKLIGHT
extern void kpd_enable_backlight(void);
extern void kpd_disable_backlight(void);
extern void kpd_backlight_handler(bool pressed, u16 linux_keycode);
#else
#define kpd_enable_backlight()		do {} while (0)
#define kpd_disable_backlight()		do {} while (0)
#define kpd_backlight_handler(pressed, linux_keycode)	do {} while (0)
#endif

#if KPD_PWRKEY_USE_EINT
#define KPD_PWRKEY_EINT           31
#define KPD_PWRKEY_DEBOUNCE       0x780 /* (0x780>>5) ms  */
#define KPD_PWRKEY_POLARITY       0 /* Low */
#define KPD_PWRKEY_SENSITIVE      0 /* Edge */
#endif

/* for META tool */
extern void kpd_set_backlight(bool onoff, void *val1, void *val2);

#endif
