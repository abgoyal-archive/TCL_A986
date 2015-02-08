

#include <asm/sn/leds.h>

void snidle(int state)
{
	if (state) {
		if (pda->idle_flag == 0) {
			/* 
			 * Turn the activity LED off.
			 */
			set_led_bits(0, LED_CPU_ACTIVITY);
		}

		pda->idle_flag = 1;
	} else {
		/* 
		 * Turn the activity LED on.
		 */
		set_led_bits(LED_CPU_ACTIVITY, LED_CPU_ACTIVITY);

		pda->idle_flag = 0;
	}
}
