

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/major.h>
#include <linux/interrupt.h>

#include <linux/parport.h>
#include <linux/parport_pc.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ciscode.h>

/*====================================================================*/

/* Module parameters */

MODULE_AUTHOR("David Hinds <dahinds@users.sourceforge.net>");
MODULE_DESCRIPTION("PCMCIA parallel port card driver");
MODULE_LICENSE("Dual MPL/GPL");

#define INT_MODULE_PARM(n, v) static int n = v; module_param(n, int, 0)

INT_MODULE_PARM(epp_mode, 1);


/*====================================================================*/

#define FORCE_EPP_MODE	0x08

typedef struct parport_info_t {
	struct pcmcia_device	*p_dev;
    int			ndev;
    struct parport	*port;
} parport_info_t;

static void parport_detach(struct pcmcia_device *p_dev);
static int parport_config(struct pcmcia_device *link);
static void parport_cs_release(struct pcmcia_device *);


static int parport_probe(struct pcmcia_device *link)
{
    parport_info_t *info;

    dev_dbg(&link->dev, "parport_attach()\n");

    /* Create new parport device */
    info = kzalloc(sizeof(*info), GFP_KERNEL);
    if (!info) return -ENOMEM;
    link->priv = info;
    info->p_dev = link;

    link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
    link->io.Attributes2 = IO_DATA_PATH_WIDTH_8;
    link->conf.Attributes = CONF_ENABLE_IRQ;
    link->conf.IntType = INT_MEMORY_AND_IO;

    return parport_config(link);
} /* parport_attach */


static void parport_detach(struct pcmcia_device *link)
{
    dev_dbg(&link->dev, "parport_detach\n");

    parport_cs_release(link);

    kfree(link->priv);
} /* parport_detach */


static int parport_config_check(struct pcmcia_device *p_dev,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc,
				void *priv_data)
{
	if ((cfg->io.nwin > 0) || (dflt->io.nwin > 0)) {
		cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt->io;
		if (epp_mode)
			p_dev->conf.ConfigIndex |= FORCE_EPP_MODE;
		p_dev->io.BasePort1 = io->win[0].base;
		p_dev->io.NumPorts1 = io->win[0].len;
		p_dev->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
		if (io->nwin == 2) {
			p_dev->io.BasePort2 = io->win[1].base;
			p_dev->io.NumPorts2 = io->win[1].len;
		}
		if (pcmcia_request_io(p_dev, &p_dev->io) != 0)
			return -ENODEV;
		return 0;
	}
	return -ENODEV;
}

static int parport_config(struct pcmcia_device *link)
{
    parport_info_t *info = link->priv;
    struct parport *p;
    int ret;

    dev_dbg(&link->dev, "parport_config\n");

    ret = pcmcia_loop_config(link, parport_config_check, NULL);
    if (ret)
	    goto failed;

    if (!link->irq)
	    goto failed;
    ret = pcmcia_request_configuration(link, &link->conf);
    if (ret)
	    goto failed;

    p = parport_pc_probe_port(link->io.BasePort1, link->io.BasePort2,
			      link->irq, PARPORT_DMA_NONE,
			      &link->dev, IRQF_SHARED);
    if (p == NULL) {
	printk(KERN_NOTICE "parport_cs: parport_pc_probe_port() at "
	       "0x%3x, irq %u failed\n", link->io.BasePort1,
	       link->irq);
	goto failed;
    }

    p->modes |= PARPORT_MODE_PCSPP;
    if (epp_mode)
	p->modes |= PARPORT_MODE_TRISTATE | PARPORT_MODE_EPP;
    info->ndev = 1;
    info->port = p;

    return 0;

failed:
    parport_cs_release(link);
    return -ENODEV;
} /* parport_config */


static void parport_cs_release(struct pcmcia_device *link)
{
	parport_info_t *info = link->priv;

	dev_dbg(&link->dev, "parport_release\n");

	if (info->ndev) {
		struct parport *p = info->port;
		parport_pc_unregister_port(p);
	}
	info->ndev = 0;

	pcmcia_disable_device(link);
} /* parport_cs_release */


static struct pcmcia_device_id parport_ids[] = {
	PCMCIA_DEVICE_FUNC_ID(3),
	PCMCIA_MFC_DEVICE_PROD_ID12(1,"Elan","Serial+Parallel Port: SP230",0x3beb8cf2,0xdb9e58bc),
	PCMCIA_DEVICE_MANF_CARD(0x0137, 0x0003),
	PCMCIA_DEVICE_NULL
};
MODULE_DEVICE_TABLE(pcmcia, parport_ids);

static struct pcmcia_driver parport_cs_driver = {
	.owner		= THIS_MODULE,
	.drv		= {
		.name	= "parport_cs",
	},
	.probe		= parport_probe,
	.remove		= parport_detach,
	.id_table	= parport_ids,
};

static int __init init_parport_cs(void)
{
	return pcmcia_register_driver(&parport_cs_driver);
}

static void __exit exit_parport_cs(void)
{
	pcmcia_unregister_driver(&parport_cs_driver);
}

module_init(init_parport_cs);
module_exit(exit_parport_cs);
