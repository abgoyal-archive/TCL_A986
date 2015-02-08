

#include <linux/module.h>
#include <linux/kernel.h>
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

MODULE_DESCRIPTION("ISDN4Linux: PCMCIA client driver for Elsa PCM cards");
MODULE_AUTHOR("Klaus Lichtenwalder");
MODULE_LICENSE("Dual MPL/GPL");


/*====================================================================*/

/* Parameters that can be set with 'insmod' */

static int protocol = 2;        /* EURO-ISDN Default */
module_param(protocol, int, 0);

/*====================================================================*/


static int elsa_cs_config(struct pcmcia_device *link) __devinit ;
static void elsa_cs_release(struct pcmcia_device *link);


static void elsa_cs_detach(struct pcmcia_device *p_dev) __devexit;

typedef struct local_info_t {
	struct pcmcia_device	*p_dev;
    int                 busy;
    int			cardnr;
} local_info_t;


static int __devinit elsa_cs_probe(struct pcmcia_device *link)
{
    local_info_t *local;

    dev_dbg(&link->dev, "elsa_cs_attach()\n");

    /* Allocate space for private device-specific data */
    local = kzalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return -ENOMEM;

    local->p_dev = link;
    link->priv = local;

    local->cardnr = -1;

    /*
      General socket configuration defaults can go here.  In this
      client, we assume very little, and rely on the CIS for almost
      everything.  In most clients, many details (i.e., number, sizes,
      and attributes of IO windows) are fixed by the nature of the
      device, and can be hard-wired here.
    */
    link->io.NumPorts1 = 8;
    link->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
    link->io.IOAddrLines = 3;

    link->conf.Attributes = CONF_ENABLE_IRQ;
    link->conf.IntType = INT_MEMORY_AND_IO;

    return elsa_cs_config(link);
} /* elsa_cs_attach */


static void __devexit elsa_cs_detach(struct pcmcia_device *link)
{
	local_info_t *info = link->priv;

	dev_dbg(&link->dev, "elsa_cs_detach(0x%p)\n", link);

	info->busy = 1;
	elsa_cs_release(link);

	kfree(info);
} /* elsa_cs_detach */


static int elsa_cs_configcheck(struct pcmcia_device *p_dev,
			       cistpl_cftable_entry_t *cf,
			       cistpl_cftable_entry_t *dflt,
			       unsigned int vcc,
			       void *priv_data)
{
	int j;

	if ((cf->io.nwin > 0) && cf->io.win[0].base) {
		printk(KERN_INFO "(elsa_cs: looks like the 96 model)\n");
		p_dev->io.BasePort1 = cf->io.win[0].base;
		if (!pcmcia_request_io(p_dev, &p_dev->io))
			return 0;
	} else {
		printk(KERN_INFO "(elsa_cs: looks like the 97 model)\n");
		for (j = 0x2f0; j > 0x100; j -= 0x10) {
			p_dev->io.BasePort1 = j;
			if (!pcmcia_request_io(p_dev, &p_dev->io))
				return 0;
		}
	}
	return -ENODEV;
}

static int __devinit elsa_cs_config(struct pcmcia_device *link)
{
    local_info_t *dev;
    int i;
    IsdnCard_t icard;

    dev_dbg(&link->dev, "elsa_config(0x%p)\n", link);
    dev = link->priv;

    i = pcmcia_loop_config(link, elsa_cs_configcheck, NULL);
    if (i != 0)
	goto failed;

    if (!link->irq)
	goto failed;

    i = pcmcia_request_configuration(link, &link->conf);
    if (i != 0)
	goto failed;

    /* Finally, report what we've done */
    dev_info(&link->dev, "index 0x%02x: ",
	    link->conf.ConfigIndex);
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	printk(", irq %d", link->irq);
    if (link->io.NumPorts1)
        printk(", io 0x%04x-0x%04x", link->io.BasePort1,
               link->io.BasePort1+link->io.NumPorts1-1);
    if (link->io.NumPorts2)
        printk(" & 0x%04x-0x%04x", link->io.BasePort2,
               link->io.BasePort2+link->io.NumPorts2-1);
    printk("\n");

    icard.para[0] = link->irq;
    icard.para[1] = link->io.BasePort1;
    icard.protocol = protocol;
    icard.typ = ISDN_CTYPE_ELSA_PCMCIA;
    
    i = hisax_init_pcmcia(link, &(((local_info_t*)link->priv)->busy), &icard);
    if (i < 0) {
    	printk(KERN_ERR "elsa_cs: failed to initialize Elsa PCMCIA %d at i/o %#x\n",
    		i, link->io.BasePort1);
    	elsa_cs_release(link);
    } else
    	((local_info_t*)link->priv)->cardnr = i;

    return 0;
failed:
    elsa_cs_release(link);
    return -ENODEV;
} /* elsa_cs_config */


static void elsa_cs_release(struct pcmcia_device *link)
{
    local_info_t *local = link->priv;

    dev_dbg(&link->dev, "elsa_cs_release(0x%p)\n", link);

    if (local) {
    	if (local->cardnr >= 0) {
    	    /* no unregister function with hisax */
	    HiSax_closecard(local->cardnr);
	}
    }

    pcmcia_disable_device(link);
} /* elsa_cs_release */

static int elsa_suspend(struct pcmcia_device *link)
{
	local_info_t *dev = link->priv;

        dev->busy = 1;

	return 0;
}

static int elsa_resume(struct pcmcia_device *link)
{
	local_info_t *dev = link->priv;

        dev->busy = 0;

	return 0;
}

static struct pcmcia_device_id elsa_ids[] = {
	PCMCIA_DEVICE_PROD_ID12("ELSA AG (Aachen, Germany)", "MicroLink ISDN/MC ", 0x983de2c4, 0x333ba257),
	PCMCIA_DEVICE_PROD_ID12("ELSA GmbH, Aachen", "MicroLink ISDN/MC ", 0x639e5718, 0x333ba257),
	PCMCIA_DEVICE_NULL
};
MODULE_DEVICE_TABLE(pcmcia, elsa_ids);

static struct pcmcia_driver elsa_cs_driver = {
	.owner		= THIS_MODULE,
	.drv		= {
		.name	= "elsa_cs",
	},
	.probe		= elsa_cs_probe,
	.remove		= __devexit_p(elsa_cs_detach),
	.id_table	= elsa_ids,
	.suspend	= elsa_suspend,
	.resume		= elsa_resume,
};

static int __init init_elsa_cs(void)
{
	return pcmcia_register_driver(&elsa_cs_driver);
}

static void __exit exit_elsa_cs(void)
{
	pcmcia_unregister_driver(&elsa_cs_driver);
}

module_init(init_elsa_cs);
module_exit(exit_elsa_cs);
