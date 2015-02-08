

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include <scsi/scsi_host.h>

#define AUTOSENSE

#define NCR5380_read(reg)		inb(port + reg)
#define NCR5380_write(reg, value)	outb(value, port + reg)

#define NCR5380_implementation_fields	unsigned int port
#define NCR5380_local_declare()		NCR5380_implementation_fields
#define NCR5380_setup(instance)		port = instance->io_port

#include <linux/delay.h>
#include "scsi.h"

#include "NCR5380.h"
#include "NCR5380.c"

#define DMX3191D_DRIVER_NAME	"dmx3191d"
#define DMX3191D_REGION_LEN	8


static struct scsi_host_template dmx3191d_driver_template = {
	.proc_name		= DMX3191D_DRIVER_NAME,
	.name			= "Domex DMX3191D",
	.queuecommand		= NCR5380_queue_command,
	.eh_abort_handler	= NCR5380_abort,
	.eh_bus_reset_handler	= NCR5380_bus_reset,
	.can_queue		= 32,
	.this_id		= 7,
	.sg_tablesize		= SG_ALL,
	.cmd_per_lun		= 2,
	.use_clustering		= DISABLE_CLUSTERING,
};

static int __devinit dmx3191d_probe_one(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	struct Scsi_Host *shost;
	unsigned long io;
	int error = -ENODEV;

	if (pci_enable_device(pdev))
		goto out;

	io = pci_resource_start(pdev, 0);
	if (!request_region(io, DMX3191D_REGION_LEN, DMX3191D_DRIVER_NAME)) {
		printk(KERN_ERR "dmx3191: region 0x%lx-0x%lx already reserved\n",
				io, io + DMX3191D_REGION_LEN);
		goto out_disable_device;
	}

	shost = scsi_host_alloc(&dmx3191d_driver_template,
			sizeof(struct NCR5380_hostdata));
	if (!shost)
		goto out_release_region;       
	shost->io_port = io;
	shost->irq = pdev->irq;

	NCR5380_init(shost, FLAG_NO_PSEUDO_DMA | FLAG_DTC3181E);

	if (request_irq(pdev->irq, NCR5380_intr, IRQF_SHARED,
				DMX3191D_DRIVER_NAME, shost)) {
		/*
		 * Steam powered scsi controllers run without an IRQ anyway
		 */
		printk(KERN_WARNING "dmx3191: IRQ %d not available - "
				    "switching to polled mode.\n", pdev->irq);
		shost->irq = SCSI_IRQ_NONE;
	}

	pci_set_drvdata(pdev, shost);

	error = scsi_add_host(shost, &pdev->dev);
	if (error)
		goto out_free_irq;

	scsi_scan_host(shost);
	return 0;

 out_free_irq:
	free_irq(shost->irq, shost);
 out_release_region:
	release_region(io, DMX3191D_REGION_LEN);
 out_disable_device:
	pci_disable_device(pdev);
 out:
	return error;
}

static void __devexit dmx3191d_remove_one(struct pci_dev *pdev)
{
	struct Scsi_Host *shost = pci_get_drvdata(pdev);

	scsi_remove_host(shost);

	NCR5380_exit(shost);

	if (shost->irq != SCSI_IRQ_NONE)
		free_irq(shost->irq, shost);
	release_region(shost->io_port, DMX3191D_REGION_LEN);
	pci_disable_device(pdev);

	scsi_host_put(shost);
}

static struct pci_device_id dmx3191d_pci_tbl[] = {
	{PCI_VENDOR_ID_DOMEX, PCI_DEVICE_ID_DOMEX_DMX3191D,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 4},
	{ }
};
MODULE_DEVICE_TABLE(pci, dmx3191d_pci_tbl);

static struct pci_driver dmx3191d_pci_driver = {
	.name		= DMX3191D_DRIVER_NAME,
	.id_table	= dmx3191d_pci_tbl,
	.probe		= dmx3191d_probe_one,
	.remove		= __devexit_p(dmx3191d_remove_one),
};

static int __init dmx3191d_init(void)
{
	return pci_register_driver(&dmx3191d_pci_driver);
}

static void __exit dmx3191d_exit(void)
{
	pci_unregister_driver(&dmx3191d_pci_driver);
}

module_init(dmx3191d_init);
module_exit(dmx3191d_exit);

MODULE_AUTHOR("Massimo Piccioni <dafastidio@libero.it>");
MODULE_DESCRIPTION("Domex DMX3191D SCSI driver");
MODULE_LICENSE("GPL");
