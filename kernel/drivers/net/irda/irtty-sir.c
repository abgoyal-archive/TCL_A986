

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <net/irda/irda.h>
#include <net/irda/irda_device.h>

#include "sir-dev.h"
#include "irtty-sir.h"

static int qos_mtt_bits = 0x03;      /* 5 ms or more */

module_param(qos_mtt_bits, int, 0);
MODULE_PARM_DESC(qos_mtt_bits, "Minimum Turn Time");

/* ------------------------------------------------------- */

/* device configuration callbacks always invoked with irda-thread context */


static int irtty_chars_in_buffer(struct sir_dev *dev)
{
	struct sirtty_cb *priv = dev->priv;

	IRDA_ASSERT(priv != NULL, return -1;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -1;);

	return tty_chars_in_buffer(priv->tty);
}


#define USBSERIAL_TX_DONE_DELAY	60

static void irtty_wait_until_sent(struct sir_dev *dev)
{
	struct sirtty_cb *priv = dev->priv;
	struct tty_struct *tty;

	IRDA_ASSERT(priv != NULL, return;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return;);

	tty = priv->tty;
	if (tty->ops->wait_until_sent) {
		tty->ops->wait_until_sent(tty, msecs_to_jiffies(100));
	}
	else {
		msleep(USBSERIAL_TX_DONE_DELAY);
	}
}


static int irtty_change_speed(struct sir_dev *dev, unsigned speed)
{
	struct sirtty_cb *priv = dev->priv;
	struct tty_struct *tty;
        struct ktermios old_termios;
	int cflag;

	IRDA_ASSERT(priv != NULL, return -1;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -1;);

	tty = priv->tty;

	mutex_lock(&tty->termios_mutex);
	old_termios = *(tty->termios);
	cflag = tty->termios->c_cflag;
	tty_encode_baud_rate(tty, speed, speed);
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);
	priv->io.speed = speed;
	mutex_unlock(&tty->termios_mutex);

	return 0;
}


static int irtty_set_dtr_rts(struct sir_dev *dev, int dtr, int rts)
{
	struct sirtty_cb *priv = dev->priv;
	int set = 0;
	int clear = 0;

	IRDA_ASSERT(priv != NULL, return -1;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -1;);

	if (rts)
		set |= TIOCM_RTS;
	else
		clear |= TIOCM_RTS;
	if (dtr)
		set |= TIOCM_DTR;
	else
		clear |= TIOCM_DTR;

	/*
	 * We can't use ioctl() because it expects a non-null file structure,
	 * and we don't have that here.
	 * This function is not yet defined for all tty driver, so
	 * let's be careful... Jean II
	 */
	IRDA_ASSERT(priv->tty->ops->tiocmset != NULL, return -1;);
	priv->tty->ops->tiocmset(priv->tty, NULL, set, clear);

	return 0;
}

/* ------------------------------------------------------- */


static int irtty_do_write(struct sir_dev *dev, const unsigned char *ptr, size_t len)
{
	struct sirtty_cb *priv = dev->priv;
	struct tty_struct *tty;
	int writelen;

	IRDA_ASSERT(priv != NULL, return -1;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -1;);

	tty = priv->tty;
	if (!tty->ops->write)
		return 0;
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	writelen = tty_write_room(tty);
	if (writelen > len)
		writelen = len;
	return tty->ops->write(tty, ptr, writelen);
}

/* ------------------------------------------------------- */

/* irda line discipline callbacks */


static void irtty_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count) 
{
	struct sir_dev *dev;
	struct sirtty_cb *priv = tty->disc_data;
	int	i;

	IRDA_ASSERT(priv != NULL, return;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return;);

	if (unlikely(count==0))		/* yes, this happens */
		return;

	dev = priv->dev;
	if (!dev) {
		IRDA_WARNING("%s(), not ready yet!\n", __func__);
		return;
	}

	for (i = 0; i < count; i++) {
		/* 
		 *  Characters received with a parity error, etc?
		 */
 		if (fp && *fp++) { 
			IRDA_DEBUG(0, "Framing or parity error!\n");
			sirdev_receive(dev, NULL, 0);	/* notify sir_dev (updating stats) */
			return;
 		}
	}

	sirdev_receive(dev, cp, count);
}

static void irtty_write_wakeup(struct tty_struct *tty) 
{
	struct sirtty_cb *priv = tty->disc_data;

	IRDA_ASSERT(priv != NULL, return;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return;);

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	if (priv->dev)
		sirdev_write_complete(priv->dev);
}

/* ------------------------------------------------------- */


static inline void irtty_stop_receiver(struct tty_struct *tty, int stop)
{
	struct ktermios old_termios;
	int cflag;

	mutex_lock(&tty->termios_mutex);
	old_termios = *(tty->termios);
	cflag = tty->termios->c_cflag;
	
	if (stop)
		cflag &= ~CREAD;
	else
		cflag |= CREAD;

	tty->termios->c_cflag = cflag;
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);
	mutex_unlock(&tty->termios_mutex);
}

/*****************************************************************/

/* serialize ldisc open/close with sir_dev */
static DEFINE_MUTEX(irtty_mutex);

/* notifier from sir_dev when irda% device gets opened (ifup) */

static int irtty_start_dev(struct sir_dev *dev)
{
	struct sirtty_cb *priv;
	struct tty_struct *tty;

	/* serialize with ldisc open/close */
	mutex_lock(&irtty_mutex);

	priv = dev->priv;
	if (unlikely(!priv || priv->magic!=IRTTY_MAGIC)) {
		mutex_unlock(&irtty_mutex);
		return -ESTALE;
	}

	tty = priv->tty;

	if (tty->ops->start)
		tty->ops->start(tty);
	/* Make sure we can receive more data */
	irtty_stop_receiver(tty, FALSE);

	mutex_unlock(&irtty_mutex);
	return 0;
}

/* notifier from sir_dev when irda% device gets closed (ifdown) */

static int irtty_stop_dev(struct sir_dev *dev)
{
	struct sirtty_cb *priv;
	struct tty_struct *tty;

	/* serialize with ldisc open/close */
	mutex_lock(&irtty_mutex);

	priv = dev->priv;
	if (unlikely(!priv || priv->magic!=IRTTY_MAGIC)) {
		mutex_unlock(&irtty_mutex);
		return -ESTALE;
	}

	tty = priv->tty;

	/* Make sure we don't receive more data */
	irtty_stop_receiver(tty, TRUE);
	if (tty->ops->stop)
		tty->ops->stop(tty);

	mutex_unlock(&irtty_mutex);

	return 0;
}

/* ------------------------------------------------------- */

static struct sir_driver sir_tty_drv = {
	.owner			= THIS_MODULE,
	.driver_name		= "sir_tty",
	.start_dev		= irtty_start_dev,
	.stop_dev		= irtty_stop_dev,
	.do_write		= irtty_do_write,
	.chars_in_buffer	= irtty_chars_in_buffer,
	.wait_until_sent	= irtty_wait_until_sent,
	.set_speed		= irtty_change_speed,
	.set_dtr_rts		= irtty_set_dtr_rts,
};

/* ------------------------------------------------------- */

static int irtty_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct irtty_info { char name[6]; } info;
	struct sir_dev *dev;
	struct sirtty_cb *priv = tty->disc_data;
	int err = 0;

	IRDA_ASSERT(priv != NULL, return -ENODEV;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -EBADR;);

	IRDA_DEBUG(3, "%s(cmd=0x%X)\n", __func__, cmd);

	dev = priv->dev;
	IRDA_ASSERT(dev != NULL, return -1;);

	switch (cmd) {
	case IRTTY_IOCTDONGLE:
		/* this call blocks for completion */
		err = sirdev_set_dongle(dev, (IRDA_DONGLE) arg);
		break;

	case IRTTY_IOCGET:
		IRDA_ASSERT(dev->netdev != NULL, return -1;);

		memset(&info, 0, sizeof(info)); 
		strncpy(info.name, dev->netdev->name, sizeof(info.name)-1);

		if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			err = -EFAULT;
		break;
	default:
		err = tty_mode_ioctl(tty, file, cmd, arg);
		break;
	}
	return err;
}


static int irtty_open(struct tty_struct *tty) 
{
	struct sir_dev *dev;
	struct sirtty_cb *priv;
	int ret = 0;

	/* Module stuff handled via irda_ldisc.owner - Jean II */

	/* First make sure we're not already connected. */
	if (tty->disc_data != NULL) {
		priv = tty->disc_data;
		if (priv && priv->magic == IRTTY_MAGIC) {
			ret = -EEXIST;
			goto out;
		}
		tty->disc_data = NULL;		/* ### */
	}

	/* stop the underlying  driver */
	irtty_stop_receiver(tty, TRUE);
	if (tty->ops->stop)
		tty->ops->stop(tty);

	tty_driver_flush_buffer(tty);
	
	/* apply mtt override */
	sir_tty_drv.qos_mtt_bits = qos_mtt_bits;

	/* get a sir device instance for this driver */
	dev = sirdev_get_instance(&sir_tty_drv, tty->name);
	if (!dev) {
		ret = -ENODEV;
		goto out;
	}

	/* allocate private device info block */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto out_put;

	priv->magic = IRTTY_MAGIC;
	priv->tty = tty;
	priv->dev = dev;

	/* serialize with start_dev - in case we were racing with ifup */
	mutex_lock(&irtty_mutex);

	dev->priv = priv;
	tty->disc_data = priv;
	tty->receive_room = 65536;

	mutex_unlock(&irtty_mutex);

	IRDA_DEBUG(0, "%s - %s: irda line discipline opened\n", __func__, tty->name);

	return 0;

out_put:
	sirdev_put_instance(dev);
out:
	return ret;
}

static void irtty_close(struct tty_struct *tty) 
{
	struct sirtty_cb *priv = tty->disc_data;

	IRDA_ASSERT(priv != NULL, return;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return;);

	/* Hm, with a dongle attached the dongle driver wants
	 * to close the dongle - which requires the use of
	 * some tty write and/or termios or ioctl operations.
	 * Are we allowed to call those when already requested
	 * to shutdown the ldisc?
	 * If not, we should somehow mark the dev being staled.
	 * Question remains, how to close the dongle in this case...
	 * For now let's assume we are granted to issue tty driver calls
	 * until we return here from the ldisc close. I'm just wondering
	 * how this behaves with hotpluggable serial hardware like
	 * rs232-pcmcia card or usb-serial...
	 *
	 * priv->tty = NULL?;
	 */

	/* we are dead now */
	tty->disc_data = NULL;

	sirdev_put_instance(priv->dev);

	/* Stop tty */
	irtty_stop_receiver(tty, TRUE);
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	if (tty->ops->stop)
		tty->ops->stop(tty);

	kfree(priv);

	IRDA_DEBUG(0, "%s - %s: irda line discipline closed\n", __func__, tty->name);
}

/* ------------------------------------------------------- */

static struct tty_ldisc_ops irda_ldisc = {
	.magic		= TTY_LDISC_MAGIC,
 	.name		= "irda",
	.flags		= 0,
	.open		= irtty_open,
	.close		= irtty_close,
	.read		= NULL,
	.write		= NULL,
	.ioctl		= irtty_ioctl,
 	.poll		= NULL,
	.receive_buf	= irtty_receive_buf,
	.write_wakeup	= irtty_write_wakeup,
	.owner		= THIS_MODULE,
};

/* ------------------------------------------------------- */

static int __init irtty_sir_init(void)
{
	int err;

	if ((err = tty_register_ldisc(N_IRDA, &irda_ldisc)) != 0)
		IRDA_ERROR("IrDA: can't register line discipline (err = %d)\n",
			   err);
	return err;
}

static void __exit irtty_sir_cleanup(void) 
{
	int err;

	if ((err = tty_unregister_ldisc(N_IRDA))) {
		IRDA_ERROR("%s(), can't unregister line discipline (err = %d)\n",
			   __func__, err);
	}
}

module_init(irtty_sir_init);
module_exit(irtty_sir_cleanup);

MODULE_AUTHOR("Dag Brattli <dagb@cs.uit.no>");
MODULE_DESCRIPTION("IrDA TTY device driver");
MODULE_ALIAS_LDISC(N_IRDA);
MODULE_LICENSE("GPL");

