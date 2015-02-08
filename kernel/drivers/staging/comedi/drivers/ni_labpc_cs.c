


#undef LABPC_DEBUG  /* debugging messages */

#include "../comedidev.h"

#include <linux/delay.h>
#include <linux/slab.h>

#include "8253.h"
#include "8255.h"
#include "comedi_fc.h"
#include "ni_labpc.h"

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>

static struct pcmcia_device *pcmcia_cur_dev;

static int labpc_attach(struct comedi_device *dev, struct comedi_devconfig *it);

static const struct labpc_board_struct labpc_cs_boards[] = {
	{
	 .name = "daqcard-1200",
	 .device_id = 0x103,	/* 0x10b is manufacturer id,
				   0x103 is device id */
	 .ai_speed = 10000,
	 .bustype = pcmcia_bustype,
	 .register_layout = labpc_1200_layout,
	 .has_ao = 1,
	 .ai_range_table = &range_labpc_1200_ai,
	 .ai_range_code = labpc_1200_ai_gain_bits,
	 .ai_range_is_unipolar = labpc_1200_is_unipolar,
	 .ai_scan_up = 0,
	 .memory_mapped_io = 0,
	 },
	/* duplicate entry, to support using alternate name */
	{
	 .name = "ni_labpc_cs",
	 .device_id = 0x103,
	 .ai_speed = 10000,
	 .bustype = pcmcia_bustype,
	 .register_layout = labpc_1200_layout,
	 .has_ao = 1,
	 .ai_range_table = &range_labpc_1200_ai,
	 .ai_range_code = labpc_1200_ai_gain_bits,
	 .ai_range_is_unipolar = labpc_1200_is_unipolar,
	 .ai_scan_up = 0,
	 .memory_mapped_io = 0,
	 },
};

#define thisboard ((const struct labpc_board_struct *)dev->board_ptr)

static struct comedi_driver driver_labpc_cs = {
	.driver_name = "ni_labpc_cs",
	.module = THIS_MODULE,
	.attach = &labpc_attach,
	.detach = &labpc_common_detach,
	.num_names = ARRAY_SIZE(labpc_cs_boards),
	.board_name = &labpc_cs_boards[0].name,
	.offset = sizeof(struct labpc_board_struct),
};

static int labpc_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	unsigned long iobase = 0;
	unsigned int irq = 0;
	struct pcmcia_device *link;

	/* allocate and initialize dev->private */
	if (alloc_private(dev, sizeof(struct labpc_private)) < 0)
		return -ENOMEM;

	/*  get base address, irq etc. based on bustype */
	switch (thisboard->bustype) {
	case pcmcia_bustype:
		link = pcmcia_cur_dev;	/* XXX hack */
		if (!link)
			return -EIO;
		iobase = link->io.BasePort1;
		irq = link->irq;
		break;
	default:
		printk("bug! couldn't determine board type\n");
		return -EINVAL;
		break;
	}
	return labpc_common_attach(dev, iobase, irq, 0);
}

/*====================================================================*/


static void labpc_config(struct pcmcia_device *link);
static void labpc_release(struct pcmcia_device *link);
static int labpc_cs_suspend(struct pcmcia_device *p_dev);
static int labpc_cs_resume(struct pcmcia_device *p_dev);


static int labpc_cs_attach(struct pcmcia_device *);
static void labpc_cs_detach(struct pcmcia_device *);



static const dev_info_t dev_info = "daqcard-1200";

struct local_info_t {
	struct pcmcia_device *link;
	int stop;
	struct bus_operations *bus;
};


static int labpc_cs_attach(struct pcmcia_device *link)
{
	struct local_info_t *local;

	dev_dbg(&link->dev, "labpc_cs_attach()\n");

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

	pcmcia_cur_dev = link;

	labpc_config(link);

	return 0;
}				/* labpc_cs_attach */


static void labpc_cs_detach(struct pcmcia_device *link)
{
	dev_dbg(&link->dev, "labpc_cs_detach\n");

	/*
	   If the device is currently configured and active, we won't
	   actually delete it yet.  Instead, it is marked so that when
	   the release() function is called, that will trigger a proper
	   detach().
	 */
	((struct local_info_t *)link->priv)->stop = 1;
	labpc_release(link);

	/* This points to the parent local_info_t struct (may be null) */
	kfree(link->priv);

}				/* labpc_cs_detach */


static int labpc_pcmcia_config_loop(struct pcmcia_device *p_dev,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc,
				void *priv_data)
{
	win_req_t *req = priv_data;
	memreq_t map;

	if (cfg->index == 0)
		return -ENODEV;

	/* Does this card need audio output? */
	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		p_dev->conf.Attributes |= CONF_ENABLE_SPKR;
		p_dev->conf.Status = CCSR_AUDIO_ENA;
	}

	/* Do we need to allocate an interrupt? */
	p_dev->conf.Attributes |= CONF_ENABLE_IRQ | CONF_ENABLE_PULSE_IRQ;

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
		if (pcmcia_request_io(p_dev, &p_dev->io) != 0)
			return -ENODEV;
	}

	if ((cfg->mem.nwin > 0) || (dflt->mem.nwin > 0)) {
		cistpl_mem_t *mem =
			(cfg->mem.nwin) ? &cfg->mem : &dflt->mem;
		req->Attributes = WIN_DATA_WIDTH_16 | WIN_MEMORY_TYPE_CM;
		req->Attributes |= WIN_ENABLE;
		req->Base = mem->win[0].host_addr;
		req->Size = mem->win[0].len;
		if (req->Size < 0x1000)
			req->Size = 0x1000;
		req->AccessSpeed = 0;
		if (pcmcia_request_window(p_dev, req, &p_dev->win))
			return -ENODEV;
		map.Page = 0;
		map.CardOffset = mem->win[0].card_addr;
		if (pcmcia_map_mem_page(p_dev, p_dev->win, &map))
			return -ENODEV;
	}
	/* If we got this far, we're cool! */
	return 0;
}


static void labpc_config(struct pcmcia_device *link)
{
	int ret;
	win_req_t req;

	dev_dbg(&link->dev, "labpc_config\n");

	ret = pcmcia_loop_config(link, labpc_pcmcia_config_loop, &req);
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
		printk(", irq %d", link->irq);
	if (link->io.NumPorts1)
		printk(", io 0x%04x-0x%04x", link->io.BasePort1,
		       link->io.BasePort1 + link->io.NumPorts1 - 1);
	if (link->io.NumPorts2)
		printk(" & 0x%04x-0x%04x", link->io.BasePort2,
		       link->io.BasePort2 + link->io.NumPorts2 - 1);
	if (link->win)
		printk(", mem 0x%06lx-0x%06lx", req.Base,
		       req.Base + req.Size - 1);
	printk("\n");

	return;

failed:
	labpc_release(link);

}				/* labpc_config */

static void labpc_release(struct pcmcia_device *link)
{
	dev_dbg(&link->dev, "labpc_release\n");

	pcmcia_disable_device(link);
}				/* labpc_release */


static int labpc_cs_suspend(struct pcmcia_device *link)
{
	struct local_info_t *local = link->priv;

	/* Mark the device as stopped, to block IO until later */
	local->stop = 1;
	return 0;
}				/* labpc_cs_suspend */

static int labpc_cs_resume(struct pcmcia_device *link)
{
	struct local_info_t *local = link->priv;

	local->stop = 0;
	return 0;
}				/* labpc_cs_resume */

/*====================================================================*/

static struct pcmcia_device_id labpc_cs_ids[] = {
	/* N.B. These IDs should match those in labpc_cs_boards (ni_labpc.c) */
	PCMCIA_DEVICE_MANF_CARD(0x010b, 0x0103),	/* daqcard-1200 */
	PCMCIA_DEVICE_NULL
};

MODULE_DEVICE_TABLE(pcmcia, labpc_cs_ids);
MODULE_AUTHOR("Frank Mori Hess <fmhess@users.sourceforge.net>");
MODULE_DESCRIPTION("Comedi driver for National Instruments Lab-PC");
MODULE_LICENSE("GPL");

struct pcmcia_driver labpc_cs_driver = {
	.probe = labpc_cs_attach,
	.remove = labpc_cs_detach,
	.suspend = labpc_cs_suspend,
	.resume = labpc_cs_resume,
	.id_table = labpc_cs_ids,
	.owner = THIS_MODULE,
	.drv = {
		.name = dev_info,
		},
};

static int __init init_labpc_cs(void)
{
	pcmcia_register_driver(&labpc_cs_driver);
	return 0;
}

static void __exit exit_labpc_cs(void)
{
	pcmcia_unregister_driver(&labpc_cs_driver);
}

int __init labpc_init_module(void)
{
	int ret;

	ret = init_labpc_cs();
	if (ret < 0)
		return ret;

	return comedi_driver_register(&driver_labpc_cs);
}

void __exit labpc_exit_module(void)
{
	exit_labpc_cs();
	comedi_driver_unregister(&driver_labpc_cs);
}

module_init(labpc_init_module);
module_exit(labpc_exit_module);
