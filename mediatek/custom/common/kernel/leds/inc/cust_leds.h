
#ifndef _CUST_LEDS_H
#define _CUST_LEDS_H

enum mt65xx_led_type
{
	MT65XX_LED_TYPE_RED = 0,
	MT65XX_LED_TYPE_GREEN,
	MT65XX_LED_TYPE_BLUE,
	MT65XX_LED_TYPE_JOGBALL,
	MT65XX_LED_TYPE_KEYBOARD,
	MT65XX_LED_TYPE_BUTTON,	
	MT65XX_LED_TYPE_LCD,
	MT65XX_LED_TYPE_TOTAL,
};

enum mt65xx_led_mode
{
	MT65XX_LED_MODE_NONE,
	MT65XX_LED_MODE_PWM,
	MT65XX_LED_MODE_GPIO,
	MT65XX_LED_MODE_PMIC,
	MT65XX_LED_MODE_CUST
};

enum mt65xx_led_pmic
{
	MT65XX_LED_PMIC_KEYBOARD,
	MT65XX_LED_PMIC_BUTTON,
	MT65XX_LED_PMIC_LCD,
};

typedef int (*cust_brightness_set)(int level, int div);

struct cust_mt65xx_led {
	char                 *name;
	enum mt65xx_led_mode  mode;
	int                   data;
};

extern struct cust_mt65xx_led *get_cust_led_list(void);

#endif

