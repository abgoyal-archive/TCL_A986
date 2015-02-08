


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <net/irda/irda.h>

#include "sir-dev.h"

#define MIN_DELAY 10	/* 10 us to be on the conservative side */

static int actisys_open(struct sir_dev *);
static int actisys_close(struct sir_dev *);
static int actisys_change_speed(struct sir_dev *, unsigned);
static int actisys_reset(struct sir_dev *);

/* These are the baudrates supported, in the order available */
/* Note : the 220L doesn't support 38400, but we will fix that below */
static unsigned baud_rates[] = { 9600, 19200, 57600, 115200, 38400 };

#define MAX_SPEEDS ARRAY_SIZE(baud_rates)

static struct dongle_driver act220l = {
	.owner		= THIS_MODULE,
	.driver_name	= "Actisys ACT-220L",
	.type		= IRDA_ACTISYS_DONGLE,
	.open		= actisys_open,
	.close		= actisys_close,
	.reset		= actisys_reset,
	.set_speed	= actisys_change_speed,
};

static struct dongle_driver act220l_plus = {
	.owner		= THIS_MODULE,
	.driver_name	= "Actisys ACT-220L+",
	.type		= IRDA_ACTISYS_PLUS_DONGLE,
	.open		= actisys_open,
	.close		= actisys_close,
	.reset		= actisys_reset,
	.set_speed	= actisys_change_speed,
};

static int __init actisys_sir_init(void)
{
	int ret;

	/* First, register an Actisys 220L dongle */
	ret = irda_register_dongle(&act220l);
	if (ret < 0)
		return ret;

	/* Now, register an Actisys 220L+ dongle */
	ret = irda_register_dongle(&act220l_plus);
	if (ret < 0) {
		irda_unregister_dongle(&act220l);
		return ret;
	}
	return 0;
}

static void __exit actisys_sir_cleanup(void)
{
	/* We have to remove both dongles */
	irda_unregister_dongle(&act220l_plus);
	irda_unregister_dongle(&act220l);
}

static int actisys_open(struct sir_dev *dev)
{
	struct qos_info *qos = &dev->qos;

	sirdev_set_dtr_rts(dev, TRUE, TRUE);

	/* Set the speeds we can accept */
	qos->baud_rate.bits &= IR_9600|IR_19200|IR_38400|IR_57600|IR_115200;

	/* Remove support for 38400 if this is not a 220L+ dongle */
	if (dev->dongle_drv->type == IRDA_ACTISYS_DONGLE)
		qos->baud_rate.bits &= ~IR_38400;

	qos->min_turn_time.bits = 0x7f; /* Needs 0.01 ms */
	irda_qos_bits_to_value(qos);

	/* irda thread waits 50 msec for power settling */

	return 0;
}

static int actisys_close(struct sir_dev *dev)
{
	/* Power off the dongle */
	sirdev_set_dtr_rts(dev, FALSE, FALSE);

	return 0;
}

static int actisys_change_speed(struct sir_dev *dev, unsigned speed)
{
	int ret = 0;
	int i = 0;

        IRDA_DEBUG(4, "%s(), speed=%d (was %d)\n", __func__,
        	speed, dev->speed);

	/* dongle was already resetted from irda_request state machine,
	 * we are in known state (dongle default)
	 */

	/* 
	 * Now, we can set the speed requested. Send RTS pulses until we
         * reach the target speed 
	 */
	for (i = 0; i < MAX_SPEEDS; i++) {
		if (speed == baud_rates[i]) {
			dev->speed = speed;
			break;
		}
		/* Set RTS low for 10 us */
		sirdev_set_dtr_rts(dev, TRUE, FALSE);
		udelay(MIN_DELAY);

		/* Set RTS high for 10 us */
		sirdev_set_dtr_rts(dev, TRUE, TRUE);
		udelay(MIN_DELAY);
	}

	/* Check if life is sweet... */
	if (i >= MAX_SPEEDS) {
		actisys_reset(dev);
		ret = -EINVAL;  /* This should not happen */
	}

	/* Basta lavoro, on se casse d'ici... */
	return ret;
}


static int actisys_reset(struct sir_dev *dev)
{
	/* Reset the dongle : set DTR low for 10 us */
	sirdev_set_dtr_rts(dev, FALSE, TRUE);
	udelay(MIN_DELAY);

	/* Go back to normal mode */
	sirdev_set_dtr_rts(dev, TRUE, TRUE);
	
	dev->speed = 9600;	/* That's the default */

	return 0;
}

MODULE_AUTHOR("Dag Brattli <dagb@cs.uit.no> - Jean Tourrilhes <jt@hpl.hp.com>");
MODULE_DESCRIPTION("ACTiSYS IR-220L and IR-220L+ dongle driver");	
MODULE_LICENSE("GPL");
MODULE_ALIAS("irda-dongle-2"); /* IRDA_ACTISYS_DONGLE */
MODULE_ALIAS("irda-dongle-3"); /* IRDA_ACTISYS_PLUS_DONGLE */

module_init(actisys_sir_init);
module_exit(actisys_sir_cleanup);
