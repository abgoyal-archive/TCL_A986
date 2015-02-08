

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <asm/system.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>
#include "hisax_cfg.h"

MODULE_DESCRIPTION("ISDN4Linux: PCMCIA client driver for Sedlbauer cards");
MODULE_AUTHOR("Marcus Niemann");
MODULE_LICENSE("Dual MPL/GPL");


/*====================================================================*/

/* Parameters that can be set with 'insmod' */

static int protocol = 2;        /* EURO-ISDN Default */
module_param(protocol, int, 0);

/*====================================================================*/


static int sedlbauer_config(struct pcmcia_device *link) __devinit ;
static void sedlbauer_release(struct pcmcia_device *link);


static void sedlbauer_detach(struct pcmcia_device *p_dev) __devexit;

typedef struct local_info_t {
	struct pcmcia_device	*p_dev;
    int			stop;
    int			cardnr;
} local_info_t;


static int __devinit sedlbauer_probe(struct pcmcia_device *link)
{
    local_info_t *local;

    dev_dbg(&link->dev, "sedlbauer_attach()\n");

    /* Allocate space for private device-specific data */
    local = kzalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return -ENOMEM;
    local->cardnr = -1;

    local->p_dev = link;
    link->priv = local;

    /*
      General socket configuration defaults can go here.  In this
      client, we assume very little, and rely on the CIS for almost
      everything.  In most clients, many details (i.e., number, sizes,
      and attributes of IO windows) are fixed by the nature of the
      device, and can be hard-wired here.
    */

    /* from old sedl_cs 
    */
    /* The io structure describes IO port mapping */
    link->io.NumPorts1 = 8;
    link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
    link->io.IOAddrLines = 3;

    link->conf.Attributes = 0;
    link->conf.IntType = INT_MEMORY_AND_IO;

    return sedlbauer_config(link);
} /* sedlbauer_attach */


static void __devexit sedlbauer_detach(struct pcmcia_device *link)
{
	dev_dbg(&link->dev, "sedlbauer_detach(0x%p)\n", link);

	((local_info_t *)link->priv)->stop = 1;
	sedlbauer_release(link);

	/* This points to the parent local_info_t struct */
	kfree(link->priv);
} /* sedlbauer_detach */

static int sedlbauer_config_check(struct pcmcia_device *p_dev,
				  cistpl_cftable_entry_t *cfg,
				  cistpl_cftable_entry_t *dflt,
				  unsigned int vcc,
				  void *priv_data)
{
	win_req_t *req = priv_data;

	if (cfg->index == 0)
		return -ENODEV;

	/* Does this card need audio output? */
	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		p_dev->conf.Attributes |= CONF_ENABLE_SPKR;
		p_dev->conf.Status = CCSR_AUDIO_ENA;
	}

	/* Use power settings for Vcc and Vpp if present */
	/*  Note that the CIS values need to be rescaled */
	if (cfg->vcc.present & (1<<CISTPL_POWER_VNOM)) {
		if (vcc != cfg->vcc.param[CISTPL_POWER_VNOM]/10000)
			return -ENODEV;
	} else if (dflt->vcc.present & (1<<CISTPL_POWER_VNOM)) {
		if (vcc != dflt->vcc.param[CISTPL_POWER_VNOM]/10000)
			return -ENODEV;
	}

	if (cfg->vpp1.present & (1<<CISTPL_POWER_VNOM))
		p_dev->conf.Vpp = cfg->vpp1.param[CISTPL_POWER_VNOM]/10000;
	else if (dflt->vpp1.present & (1<<CISTPL_POWER_VNOM))
		p_dev->conf.Vpp = dflt->vpp1.param[CISTPL_POWER_VNOM]/10000;

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

	/*
	  Now set up a common memory window, if needed.  There is room
	  in the struct pcmcia_device structure for one memory window handle,
	  but if the base addresses need to be saved, or if multiple
	  windows are needed, the info should go in the private data
	  structure for this device.

	  Note that the memory window base is a physical address, and
	  needs to be mapped to virtual space with ioremap() before it
	  is used.
	*/
	if ((cfg->mem.nwin > 0) || (dflt->mem.nwin > 0)) {
		cistpl_mem_t *mem = (cfg->mem.nwin) ? &cfg->mem : &dflt->mem;
		memreq_t map;
		req->Attributes = WIN_DATA_WIDTH_16|WIN_MEMORY_TYPE_CM;
		req->Attributes |= WIN_ENABLE;
		req->Base = mem->win[0].host_addr;
		req->Size = mem->win[0].len;
		req->AccessSpeed = 0;
		if (pcmcia_request_window(p_dev, req, &p_dev->win) != 0)
			return -ENODEV;
		map.Page = 0;
		map.CardOffset = mem->win[0].card_addr;
		if (pcmcia_map_mem_page(p_dev, p_dev->win, &map) != 0)
			return -ENODEV;
	}
	return 0;
}



static int __devinit sedlbauer_config(struct pcmcia_device *link)
{
    win_req_t *req;
    int ret;
    IsdnCard_t  icard;

    dev_dbg(&link->dev, "sedlbauer_config(0x%p)\n", link);

    req = kzalloc(sizeof(win_req_t), GFP_KERNEL);
    if (!req)
	    return -ENOMEM;

    /*
      In this loop, we scan the CIS for configuration table entries,
      each of which describes a valid card configuration, including
      voltage, IO window, memory window, and interrupt settings.

      We make no assumptions about the card to be configured: we use
      just the information available in the CIS.  In an ideal world,
      this would work for any PCMCIA card, but it requires a complete
      and accurate CIS.  In practice, a driver usually "knows" most of
      these things without consulting the CIS, and most client drivers
      will only use the CIS to fill in implementation-defined details.
    */
    ret = pcmcia_loop_config(link, sedlbauer_config_check, req);
    if (ret)
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
    dev_info(&link->dev, "index 0x%02x:",
	   link->conf.ConfigIndex);
    if (link->conf.Vpp)
	printk(", Vpp %d.%d", link->conf.Vpp/10, link->conf.Vpp%10);
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	printk(", irq %d", link->irq);
    if (link->io.NumPorts1)
	printk(", io 0x%04x-0x%04x", link->io.BasePort1,
	       link->io.BasePort1+link->io.NumPorts1-1);
    if (link->io.NumPorts2)
	printk(" & 0x%04x-0x%04x", link->io.BasePort2,
	       link->io.BasePort2+link->io.NumPorts2-1);
    if (link->win)
	printk(", mem 0x%06lx-0x%06lx", req->Base,
	       req->Base+req->Size-1);
    printk("\n");

    icard.para[0] = link->irq;
    icard.para[1] = link->io.BasePort1;
    icard.protocol = protocol;
    icard.typ = ISDN_CTYPE_SEDLBAUER_PCMCIA;
    
    ret = hisax_init_pcmcia(link, 
			    &(((local_info_t *)link->priv)->stop), &icard);
    if (ret < 0) {
	printk(KERN_ERR "sedlbauer_cs: failed to initialize SEDLBAUER PCMCIA %d at i/o %#x\n",
		ret, link->io.BasePort1);
    	sedlbauer_release(link);
	return -ENODEV;
    } else
	((local_info_t *)link->priv)->cardnr = ret;

    return 0;

failed:
    sedlbauer_release(link);
    return -ENODEV;

} /* sedlbauer_config */


static void sedlbauer_release(struct pcmcia_device *link)
{
    local_info_t *local = link->priv;
    dev_dbg(&link->dev, "sedlbauer_release(0x%p)\n", link);

    if (local) {
    	if (local->cardnr >= 0) {
    	    /* no unregister function with hisax */
	    HiSax_closecard(local->cardnr);
	}
    }

    pcmcia_disable_device(link);
} /* sedlbauer_release */

static int sedlbauer_suspend(struct pcmcia_device *link)
{
	local_info_t *dev = link->priv;

	dev->stop = 1;

	return 0;
}

static int sedlbauer_resume(struct pcmcia_device *link)
{
	local_info_t *dev = link->priv;

	dev->stop = 0;

	return 0;
}


static struct pcmcia_device_id sedlbauer_ids[] = {
	PCMCIA_DEVICE_PROD_ID123("SEDLBAUER", "speed star II", "V 3.1", 0x81fb79f5, 0xf3612e1d, 0x6b95c78a),
	PCMCIA_DEVICE_PROD_ID123("SEDLBAUER", "ISDN-Adapter", "4D67", 0x81fb79f5, 0xe4e9bc12, 0x397b7e90),
	PCMCIA_DEVICE_PROD_ID123("SEDLBAUER", "ISDN-Adapter", "4D98", 0x81fb79f5, 0xe4e9bc12, 0x2e5c7fce),
	PCMCIA_DEVICE_PROD_ID123("SEDLBAUER", "ISDN-Adapter", " (C) 93-94 VK", 0x81fb79f5, 0xe4e9bc12, 0x8db143fe),
	PCMCIA_DEVICE_PROD_ID123("SEDLBAUER", "ISDN-Adapter", " (c) 93-95 VK", 0x81fb79f5, 0xe4e9bc12, 0xb391ab4c),
	PCMCIA_DEVICE_PROD_ID12("HST High Soft Tech GmbH", "Saphir II B", 0xd79e0b84, 0x21d083ae),
/*	PCMCIA_DEVICE_PROD_ID1234("SEDLBAUER", 0x81fb79f5), */ /* too generic*/
	PCMCIA_DEVICE_NULL
};
MODULE_DEVICE_TABLE(pcmcia, sedlbauer_ids);

static struct pcmcia_driver sedlbauer_driver = {
	.owner		= THIS_MODULE,
	.drv		= {
		.name	= "sedlbauer_cs",
	},
	.probe		= sedlbauer_probe,
	.remove		= __devexit_p(sedlbauer_detach),
	.id_table	= sedlbauer_ids,
	.suspend	= sedlbauer_suspend,
	.resume		= sedlbauer_resume,
};

static int __init init_sedlbauer_cs(void)
{
	return pcmcia_register_driver(&sedlbauer_driver);
}

static void __exit exit_sedlbauer_cs(void)
{
	pcmcia_unregister_driver(&sedlbauer_driver);
}

module_init(init_sedlbauer_cs);
module_exit(exit_sedlbauer_cs);
