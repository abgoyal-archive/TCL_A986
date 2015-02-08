
#include <cust_leds.h>
#include <mach/mt6573_pwm.h>
#include <mach/mt6573_gpio.h>
#include <linux/delay.h>

extern int mtkfb_set_backlight_level(unsigned int level);
extern int mtkfb_set_backlight_pwm(int div);


unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;

   // mapped_level = level/16;
    mapped_level = level/4;
    if(mapped_level > 58)
   {
	mapped_level = 58;
   }

    return mapped_level;
}
static   volatile  unsigned int mapped_level_before = 0;
unsigned int Cust_SetBacklight(int level, int div)
{
	#if 0
    mtkfb_set_backlight_pwm(div);
    mtkfb_set_backlight_level(brightness_mapping(level));
    return 0;
	#endif
	unsigned int i;
	printk("power level here ,libin");
	#if 0
	mt_set_gpio_mode(100,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(100,GPIO_PULL_ENABLE);
	//---
	mt_set_gpio_dir(100,GPIO_DIR_OUT); // output
//	mt_set_gpio_out(100,GPIO_OUT_ONE); // high
	#endif
    unsigned int mapped_level;

    mapped_level = level/16;
	if(mapped_level_before == mapped_level)
		return;
		if(mapped_level == 0)
			{
			mt_set_gpio_out(100,GPIO_OUT_ZERO); // high
			mdelay(4);
				}

	else if(mapped_level < mapped_level_before)
	{

	mt_set_gpio_out(100,GPIO_OUT_ONE); // high

	udelay(40);
for(i = 0;i<(mapped_level_before - mapped_level);i++)
{
	mt_set_gpio_out(100,GPIO_OUT_ONE); // high
	udelay(1);
	mt_set_gpio_out(100,GPIO_OUT_ZERO); // high
	udelay(1);
}
	mt_set_gpio_out(100,GPIO_OUT_ONE); // high
	}
	else if(mapped_level > mapped_level_before)
			{
			mt_set_gpio_out(100,GPIO_OUT_ZERO); // high
			mdelay(4);
			mt_set_gpio_out(100,GPIO_OUT_ONE); // high
			udelay(40);
		for(i = 0;i<(16 - mapped_level);i++)
		{
			mt_set_gpio_out(100,GPIO_OUT_ONE); // high
			udelay(1);
			mt_set_gpio_out(100,GPIO_OUT_ZERO); // high
			udelay(1);
		}
			mt_set_gpio_out(100,GPIO_OUT_ONE); // high
			}
	else
	   return; // high


	mapped_level_before = mapped_level;
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_PWM, PWM1},
	{"green",             MT65XX_LED_MODE_NONE, -1},
	{"blue",              MT65XX_LED_MODE_NONE, -1},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1},
	{"button-backlight",  MT65XX_LED_MODE_PWM, PWM7},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST, (int)Cust_SetBacklight},
	{"lcd-backlight",     MT65XX_LED_MODE_PWM, PWM2},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

