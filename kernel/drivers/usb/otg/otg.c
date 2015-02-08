

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/usb/otg.h>

static struct otg_transceiver *xceiv;

struct otg_transceiver *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

void otg_put_transceiver(struct otg_transceiver *x)
{
	if (x)
		put_device(x->dev);
}
EXPORT_SYMBOL(otg_put_transceiver);

int otg_set_transceiver(struct otg_transceiver *x)
{
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);
