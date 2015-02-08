

#ifndef __BFA_DEFS_LED_H__
#define __BFA_DEFS_LED_H__

#define	BFA_LED_MAX_NUM		3

enum bfa_led_op {
	BFA_LED_OFF   = 0,
	BFA_LED_ON    = 1,
	BFA_LED_FLICK = 2,
	BFA_LED_BLINK = 3,
};

enum bfa_led_color {
	BFA_LED_GREEN = 0,
	BFA_LED_AMBER = 1,
};

#endif /* __BFA_DEFS_LED_H__ */
