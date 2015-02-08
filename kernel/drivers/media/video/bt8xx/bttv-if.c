

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "bttvp.h"

EXPORT_SYMBOL(bttv_get_pcidev);
EXPORT_SYMBOL(bttv_gpio_enable);
EXPORT_SYMBOL(bttv_read_gpio);
EXPORT_SYMBOL(bttv_write_gpio);

/* ----------------------------------------------------------------------- */
/* Exported functions - for other modules which want to access the         */
/*                      gpio ports (IR for example)                        */
/*                      see bttv.h for comments                            */

struct pci_dev* bttv_get_pcidev(unsigned int card)
{
	if (card >= bttv_num)
		return NULL;
	if (!bttvs[card])
		return NULL;

	return bttvs[card]->c.pci;
}


int bttv_gpio_enable(unsigned int card, unsigned long mask, unsigned long data)
{
	struct bttv *btv;

	if (card >= bttv_num) {
		return -EINVAL;
	}

	btv = bttvs[card];
	if (!btv)
		return -ENODEV;

	gpio_inout(mask,data);
	if (bttv_gpio)
		bttv_gpio_tracking(btv,"extern enable");
	return 0;
}

int bttv_read_gpio(unsigned int card, unsigned long *data)
{
	struct bttv *btv;

	if (card >= bttv_num) {
		return -EINVAL;
	}

	btv = bttvs[card];
	if (!btv)
		return -ENODEV;

	if(btv->shutdown) {
		return -ENODEV;
	}

	*data = gpio_read();
	return 0;
}

int bttv_write_gpio(unsigned int card, unsigned long mask, unsigned long data)
{
	struct bttv *btv;

	if (card >= bttv_num) {
		return -EINVAL;
	}

	btv = bttvs[card];
	if (!btv)
		return -ENODEV;

	gpio_bits(mask,data);
	if (bttv_gpio)
		bttv_gpio_tracking(btv,"extern write");
	return 0;
}

