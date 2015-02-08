

#include "pcmcia.h"

#include <linux/ssb/ssb.h>
#include <linux/slab.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ciscode.h>
#include <pcmcia/ds.h>
#include <pcmcia/cisreg.h>


static /*const */ struct pcmcia_device_id b43_pcmcia_tbl[] = {
	PCMCIA_DEVICE_MANF_CARD(0x2D0, 0x448),
	PCMCIA_DEVICE_MANF_CARD(0x2D0, 0x476),
	PCMCIA_DEVICE_NULL,
};

MODULE_DEVICE_TABLE(pcmcia, b43_pcmcia_tbl);

#ifdef CONFIG_PM
static int b43_pcmcia_suspend(struct pcmcia_device *dev)
{
	struct ssb_bus *ssb = dev->priv;

	return ssb_bus_suspend(ssb);
}

static int b43_pcmcia_resume(struct pcmcia_device *dev)
{
	struct ssb_bus *ssb = dev->priv;

	return ssb_bus_resume(ssb);
}
#else /* CONFIG_PM */
# define b43_pcmcia_suspend		NULL
# define b43_pcmcia_resume		NULL
#endif /* CONFIG_PM */

static int __devinit b43_pcmcia_probe(struct pcmcia_device *dev)
{
	struct ssb_bus *ssb;
	win_req_t win;
	memreq_t mem;
	int err = -ENOMEM;
	int res = 0;

	ssb = kzalloc(sizeof(*ssb), GFP_KERNEL);
	if (!ssb)
		goto out_error;

	err = -ENODEV;

	dev->conf.Attributes = CONF_ENABLE_IRQ;
	dev->conf.IntType = INT_MEMORY_AND_IO;

	dev->io.BasePort2 = 0;
	dev->io.NumPorts2 = 0;
	dev->io.Attributes2 = 0;

	win.Attributes = WIN_ADDR_SPACE_MEM | WIN_MEMORY_TYPE_CM |
			 WIN_ENABLE | WIN_DATA_WIDTH_16 |
			 WIN_USE_WAIT;
	win.Base = 0;
	win.Size = SSB_CORE_SIZE;
	win.AccessSpeed = 250;
	res = pcmcia_request_window(dev, &win, &dev->win);
	if (res != 0)
		goto err_kfree_ssb;

	mem.CardOffset = 0;
	mem.Page = 0;
	res = pcmcia_map_mem_page(dev, dev->win, &mem);
	if (res != 0)
		goto err_disable;

	if (!dev->irq)
		goto err_disable;

	res = pcmcia_request_configuration(dev, &dev->conf);
	if (res != 0)
		goto err_disable;

	err = ssb_bus_pcmciabus_register(ssb, dev, win.Base);
	if (err)
		goto err_disable;
	dev->priv = ssb;

	return 0;

err_disable:
	pcmcia_disable_device(dev);
err_kfree_ssb:
	kfree(ssb);
out_error:
	printk(KERN_ERR "b43-pcmcia: Initialization failed (%d, %d)\n",
	       res, err);
	return err;
}

static void __devexit b43_pcmcia_remove(struct pcmcia_device *dev)
{
	struct ssb_bus *ssb = dev->priv;

	ssb_bus_unregister(ssb);
	pcmcia_disable_device(dev);
	kfree(ssb);
	dev->priv = NULL;
}

static struct pcmcia_driver b43_pcmcia_driver = {
	.owner		= THIS_MODULE,
	.drv		= {
				.name = "b43-pcmcia",
			},
	.id_table	= b43_pcmcia_tbl,
	.probe		= b43_pcmcia_probe,
	.remove		= __devexit_p(b43_pcmcia_remove),
	.suspend	= b43_pcmcia_suspend,
	.resume		= b43_pcmcia_resume,
};

int b43_pcmcia_init(void)
{
	return pcmcia_register_driver(&b43_pcmcia_driver);
}

void b43_pcmcia_exit(void)
{
	pcmcia_unregister_driver(&b43_pcmcia_driver);
}
