

#ifndef __INCLUDE_ATMEL_PWM_BL_H
#define __INCLUDE_ATMEL_PWM_BL_H

struct atmel_pwm_bl_platform_data {
	unsigned int pwm_channel;
	unsigned int pwm_frequency;
	unsigned int pwm_compare_max;
	unsigned int pwm_duty_max;
	unsigned int pwm_duty_min;
	unsigned int pwm_active_low;
	int gpio_on;
	unsigned int on_active_low;
};

#endif /* __INCLUDE_ATMEL_PWM_BL_H */
