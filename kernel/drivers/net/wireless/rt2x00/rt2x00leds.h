


#ifndef RT2X00LEDS_H
#define RT2X00LEDS_H

enum led_type {
	LED_TYPE_RADIO,
	LED_TYPE_ASSOC,
	LED_TYPE_ACTIVITY,
	LED_TYPE_QUALITY,
};

struct rt2x00_led {
	struct rt2x00_dev *rt2x00dev;
	struct led_classdev led_dev;

	enum led_type type;
	unsigned int flags;
#define LED_INITIALIZED		( 1 << 0 )
#define LED_REGISTERED		( 1 << 1 )
};

#endif /* RT2X00LEDS_H */
