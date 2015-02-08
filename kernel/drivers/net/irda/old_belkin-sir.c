

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <net/irda/irda.h>
// #include <net/irda/irda_device.h>

#include "sir-dev.h"


static int old_belkin_open(struct sir_dev *dev);
static int old_belkin_close(struct sir_dev *dev);
static int old_belkin_change_speed(struct sir_dev *dev, unsigned speed);
static int old_belkin_reset(struct sir_dev *dev);

static struct dongle_driver old_belkin = {
	.owner		= THIS_MODULE,
	.driver_name	= "Old Belkin SmartBeam",
	.type		= IRDA_OLD_BELKIN_DONGLE,
	.open		= old_belkin_open,
	.close		= old_belkin_close,
	.reset		= old_belkin_reset,
	.set_speed	= old_belkin_change_speed,
};

static int __init old_belkin_sir_init(void)
{
	return irda_register_dongle(&old_belkin);
}

static void __exit old_belkin_sir_cleanup(void)
{
	irda_unregister_dongle(&old_belkin);
}

static int old_belkin_open(struct sir_dev *dev)
{
	struct qos_info *qos = &dev->qos;

	IRDA_DEBUG(2, "%s()\n", __func__);

	/* Power on dongle */
	sirdev_set_dtr_rts(dev, TRUE, TRUE);

	/* Not too fast, please... */
	qos->baud_rate.bits &= IR_9600;
	/* Needs at least 10 ms (totally wild guess, can do probably better) */
	qos->min_turn_time.bits = 0x01;
	irda_qos_bits_to_value(qos);

	/* irda thread waits 50 msec for power settling */

	return 0;
}

static int old_belkin_close(struct sir_dev *dev)
{
	IRDA_DEBUG(2, "%s()\n", __func__);

	/* Power off dongle */
	sirdev_set_dtr_rts(dev, FALSE, FALSE);

	return 0;
}

static int old_belkin_change_speed(struct sir_dev *dev, unsigned speed)
{
	IRDA_DEBUG(2, "%s()\n", __func__);

	dev->speed = 9600;
	return (speed==dev->speed) ? 0 : -EINVAL;
}

static int old_belkin_reset(struct sir_dev *dev)
{
	IRDA_DEBUG(2, "%s()\n", __func__);

	/* This dongles speed "defaults" to 9600 bps ;-) */
	dev->speed = 9600;

	return 0;
}

MODULE_AUTHOR("Jean Tourrilhes <jt@hpl.hp.com>");
MODULE_DESCRIPTION("Belkin (old) SmartBeam dongle driver");	
MODULE_LICENSE("GPL");
MODULE_ALIAS("irda-dongle-7"); /* IRDA_OLD_BELKIN_DONGLE */

module_init(old_belkin_sir_init);
module_exit(old_belkin_sir_cleanup);
