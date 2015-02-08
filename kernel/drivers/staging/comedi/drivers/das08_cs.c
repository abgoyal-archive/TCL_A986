

#include "../comedidev.h"

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "das08.h"

/* pcmcia includes */
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

static struct pcmcia_device *cur_dev;

#define thisboard ((const struct das08_board_struct *)dev->board_ptr)

static int das08_cs_attach(struct comedi_device *dev,
			   struct comedi_devconfig *it);

static struct comedi_driver driver_das08_cs = {
	.driver_name = "das08_cs",
	.module = THIS_MODULE,
	.attach = das08_cs_attach,
	.detach = das08_common_detach,
	.board_name = &das08_cs_boards[0].name,
	.num_names = ARRAY_SIZE(das08_cs_boards),
	.offset = sizeof(struct das08_board_struct),
};

static int das08_cs_attach(struct comedi_device *dev,
			   struct comedi_devconfig *it)
{
	int ret;
	unsigned long iobase;
	struct pcmcia_device *link = cur_dev;	/*  XXX hack */

	ret = alloc_private(dev, sizeof(struct das08_private_struct));
	if (ret < 0)
		return ret;

	printk("comedi%d: das08_cs: ", dev->minor);
	/*  deal with a pci board */

	if (thisboard->bustype == pcmcia) {
		if (link == NULL) {
			printk(" no pcmcia cards found\n");
			return -EIO;
		}
		iobase = link->io.BasePort1;
	} else {
		printk(" bug! board does not have PCMCIA bustype\n");
		return -EINVAL;
	}

	printk("\n");

	return das08_common_attach(dev, iobase);
}


static void das08_pcmcia_config(struct pcmcia_device *link);
static void das08_pcmcia_release(struct pcmcia_device *link);
static int das08_pcmcia_suspend(struct pcmcia_device *p_dev);
static int das08_pcmcia_resume(struct pcmcia_device *p_dev);


static int das08_pcmcia_attach(struct pcmcia_device *);
static void das08_pcmcia_detach(struct pcmcia_device *);



static const dev_info_t dev_info = "pcm-das08";

struct local_info_t {
	struct pcmcia_device *link;
	int stop;
	struct bus_operations *bus;
};


static int das08_pcmcia_attach(struct pcmcia_device *link)
{
	struct local_info_t *local;

	dev_dbg(&link->dev, "das08_pcmcia_attach()\n");

	/* Allocate space for private device-specific data */
	local = kzalloc(sizeof(struct local_info_t), GFP_KERNEL);
	if (!local)
		return -ENOMEM;
	local->link = link;
	link->priv = local;

	/*
	   General socket configuration defaults can go here.  In this
	   client, we assume very little, and rely on the CIS for almost
	   everything.  In most clients, many details (i.e., number, sizes,
	   and attributes of IO windows) are fixed by the nature of the
	   device, and can be hard-wired here.
	 */
	link->conf.Attributes = 0;
	link->conf.IntType = INT_MEMORY_AND_IO;

	cur_dev = link;

	das08_pcmcia_config(link);

	return 0;
}				/* das08_pcmcia_attach */


static void das08_pcmcia_detach(struct pcmcia_device *link)
{

	dev_dbg(&link->dev, "das08_pcmcia_detach\n");

	((struct local_info_t *)link->priv)->stop = 1;
	das08_pcmcia_release(link);

	/* This points to the parent struct local_info_t struct */
	if (link->priv)
		kfree(link->priv);

}				/* das08_pcmcia_detach */


static int das08_pcmcia_config_loop(struct pcmcia_device *p_dev,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc,
				void *priv_data)
{
	if (cfg->index == 0)
		return -ENODEV;

	/* Do we need to allocate an interrupt? */
	p_dev->conf.Attributes |= CONF_ENABLE_IRQ;

	/* IO window settings */
	p_dev->io.NumPorts1 = p_dev->io.NumPorts2 = 0;
	if ((cfg->io.nwin > 0) || (dflt->io.nwin > 0)) {
		cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt->io;
		p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
		if (!(io->flags & CISTPL_IO_8BIT))
			p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
		if (!(io->flags & CISTPL_IO_16BIT))
			p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
		p_dev->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
		p_dev->io.BasePort1 = io->win[0].base;
		p_dev->io.NumPorts1 = io->win[0].len;
		if (io->nwin > 1) {
			p_dev->io.Attributes2 = p_dev->io.Attributes1;
			p_dev->io.BasePort2 = io->win[1].base;
			p_dev->io.NumPorts2 = io->win[1].len;
		}
		/* This reserves IO space but doesn't actually enable it */
		return pcmcia_request_io(p_dev, &p_dev->io);
	}
	return 0;
}



static void das08_pcmcia_config(struct pcmcia_device *link)
{
	int ret;

	dev_dbg(&link->dev, "das08_pcmcia_config\n");

	ret = pcmcia_loop_config(link, das08_pcmcia_config_loop, NULL);
	if (ret) {
		dev_warn(&link->dev, "no configuration found\n");
		goto failed;
	}

	if (!link->irq)
		goto failed;

	/*
	   This actually configures the PCMCIA socket -- setting up
	   the I/O windows and the interrupt mapping, and putting the
	   card and host interface into "Memory and IO" mode.
	 */
	ret = pcmcia_request_configuration(link, &link->conf);
	if (ret)
		goto failed;

	/* Finally, report what we've done */
	dev_info(&link->dev, "index 0x%02x", link->conf.ConfigIndex);
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
		printk(", irq %u", link->irq);
	if (link->io.NumPorts1)
		printk(", io 0x%04x-0x%04x", link->io.BasePort1,
		       link->io.BasePort1 + link->io.NumPorts1 - 1);
	if (link->io.NumPorts2)
		printk(" & 0x%04x-0x%04x", link->io.BasePort2,
		       link->io.BasePort2 + link->io.NumPorts2 - 1);
	printk("\n");

	return;

failed:
	das08_pcmcia_release(link);

}				/* das08_pcmcia_config */


static void das08_pcmcia_release(struct pcmcia_device *link)
{
	dev_dbg(&link->dev, "das08_pcmcia_release\n");
	pcmcia_disable_device(link);
}				/* das08_pcmcia_release */


static int das08_pcmcia_suspend(struct pcmcia_device *link)
{
	struct local_info_t *local = link->priv;
	/* Mark the device as stopped, to block IO until later */
	local->stop = 1;

	return 0;
}				/* das08_pcmcia_suspend */

static int das08_pcmcia_resume(struct pcmcia_device *link)
{
	struct local_info_t *local = link->priv;

	local->stop = 0;
	return 0;
}				/* das08_pcmcia_resume */

/*====================================================================*/

static struct pcmcia_device_id das08_cs_id_table[] = {
	PCMCIA_DEVICE_MANF_CARD(0x01c5, 0x4001),
	PCMCIA_DEVICE_NULL
};

MODULE_DEVICE_TABLE(pcmcia, das08_cs_id_table);
MODULE_AUTHOR("David A. Schleef <ds@schleef.org>, "
	      "Frank Mori Hess <fmhess@users.sourceforge.net>");
MODULE_DESCRIPTION("Comedi driver for ComputerBoards DAS-08 PCMCIA boards");
MODULE_LICENSE("GPL");

struct pcmcia_driver das08_cs_driver = {
	.probe = das08_pcmcia_attach,
	.remove = das08_pcmcia_detach,
	.suspend = das08_pcmcia_suspend,
	.resume = das08_pcmcia_resume,
	.id_table = das08_cs_id_table,
	.owner = THIS_MODULE,
	.drv = {
		.name = dev_info,
		},
};

static int __init init_das08_pcmcia_cs(void)
{
	pcmcia_register_driver(&das08_cs_driver);
	return 0;
}

static void __exit exit_das08_pcmcia_cs(void)
{
	pr_debug("das08_pcmcia_cs: unloading\n");
	pcmcia_unregister_driver(&das08_cs_driver);
}

static int __init das08_cs_init_module(void)
{
	int ret;

	ret = init_das08_pcmcia_cs();
	if (ret < 0)
		return ret;

	return comedi_driver_register(&driver_das08_cs);
}

static void __exit das08_cs_exit_module(void)
{
	exit_das08_pcmcia_cs();
	comedi_driver_unregister(&driver_das08_cs);
}

module_init(das08_cs_init_module);
module_exit(das08_cs_exit_module);
