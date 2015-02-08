

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <net/irda/irda.h>

#include "sir-dev.h"

static int esi_open(struct sir_dev *);
static int esi_close(struct sir_dev *);
static int esi_change_speed(struct sir_dev *, unsigned);
static int esi_reset(struct sir_dev *);

static struct dongle_driver esi = {
	.owner		= THIS_MODULE,
	.driver_name	= "JetEye PC ESI-9680 PC",
	.type		= IRDA_ESI_DONGLE,
	.open		= esi_open,
	.close		= esi_close,
	.reset		= esi_reset,
	.set_speed	= esi_change_speed,
};

static int __init esi_sir_init(void)
{
	return irda_register_dongle(&esi);
}

static void __exit esi_sir_cleanup(void)
{
	irda_unregister_dongle(&esi);
}

static int esi_open(struct sir_dev *dev)
{
	struct qos_info *qos = &dev->qos;

	/* Power up and set dongle to 9600 baud */
	sirdev_set_dtr_rts(dev, FALSE, TRUE);

	qos->baud_rate.bits &= IR_9600|IR_19200|IR_115200;
	qos->min_turn_time.bits = 0x01; /* Needs at least 10 ms */
	irda_qos_bits_to_value(qos);

	/* irda thread waits 50 msec for power settling */

	return 0;
}

static int esi_close(struct sir_dev *dev)
{
	/* Power off dongle */
	sirdev_set_dtr_rts(dev, FALSE, FALSE);

	return 0;
}

static int esi_change_speed(struct sir_dev *dev, unsigned speed)
{
	int ret = 0;
	int dtr, rts;
	
	switch (speed) {
	case 19200:
		dtr = TRUE;
		rts = FALSE;
		break;
	case 115200:
		dtr = rts = TRUE;
		break;
	default:
		ret = -EINVAL;
		speed = 9600;
		/* fall through */
	case 9600:
		dtr = FALSE;
		rts = TRUE;
		break;
	}

	/* Change speed of dongle */
	sirdev_set_dtr_rts(dev, dtr, rts);
	dev->speed = speed;

	return ret;
}

static int esi_reset(struct sir_dev *dev)
{
	sirdev_set_dtr_rts(dev, FALSE, FALSE);

	/* Hm, the old esi-driver left the dongle unpowered relying on
	 * the following speed change to repower. This might work for
	 * the esi because we only need the modem lines. However, now the
	 * general rule is reset must bring the dongle to some working
	 * well-known state because speed change might write to registers.
	 * The old esi-driver didn't any delay here - let's hope it' fine.
	 */

	sirdev_set_dtr_rts(dev, FALSE, TRUE);
	dev->speed = 9600;

	return 0;
}

MODULE_AUTHOR("Dag Brattli <dagb@cs.uit.no>");
MODULE_DESCRIPTION("Extended Systems JetEye PC dongle driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("irda-dongle-1"); /* IRDA_ESI_DONGLE */

module_init(esi_sir_init);
module_exit(esi_sir_cleanup);

