

#include "aic7xxx_osm.h"

#include <linux/device.h>
#include <linux/eisa.h>

int
aic7770_map_registers(struct ahc_softc *ahc, u_int port)
{
	/*
	 * Lock out other contenders for our i/o space.
	 */
	if (!request_region(port, AHC_EISA_IOSIZE, "aic7xxx"))
		return (ENOMEM);
	ahc->tag = BUS_SPACE_PIO;
	ahc->bsh.ioport = port;
	return (0);
}

int
aic7770_map_int(struct ahc_softc *ahc, u_int irq)
{
	int error;
	int shared;

	shared = 0;
	if ((ahc->flags & AHC_EDGE_INTERRUPT) == 0)
		shared = IRQF_SHARED;

	error = request_irq(irq, ahc_linux_isr, shared, "aic7xxx", ahc);
	if (error == 0)
		ahc->platform_data->irq = irq;
	
	return (-error);
}

static int
aic7770_probe(struct device *dev)
{
	struct eisa_device *edev = to_eisa_device(dev);
	u_int eisaBase = edev->base_addr+AHC_EISA_SLOT_OFFSET;
	struct	ahc_softc *ahc;
	char	buf[80];
	char   *name;
	int	error;

	sprintf(buf, "ahc_eisa:%d", eisaBase >> 12);
	name = malloc(strlen(buf) + 1, M_DEVBUF, M_NOWAIT);
	if (name == NULL)
		return (ENOMEM);
	strcpy(name, buf);
	ahc = ahc_alloc(&aic7xxx_driver_template, name);
	if (ahc == NULL)
		return (ENOMEM);
	error = aic7770_config(ahc, aic7770_ident_table + edev->id.driver_data,
			       eisaBase);
	if (error != 0) {
		ahc->bsh.ioport = 0;
		ahc_free(ahc);
		return (error);
	}

 	dev_set_drvdata(dev, ahc);

	error = ahc_linux_register_host(ahc, &aic7xxx_driver_template);
	return (error);
}

static int
aic7770_remove(struct device *dev)
{
	struct ahc_softc *ahc = dev_get_drvdata(dev);
	u_long s;

	if (ahc->platform_data && ahc->platform_data->host)
			scsi_remove_host(ahc->platform_data->host);

	ahc_lock(ahc, &s);
	ahc_intr_enable(ahc, FALSE);
	ahc_unlock(ahc, &s);

	ahc_free(ahc);
	return 0;
}
 
static struct eisa_device_id aic7770_ids[] = {
	{ "ADP7771", 0 }, /* AHA 274x */
	{ "ADP7756", 1 }, /* AHA 284x BIOS enabled */
	{ "ADP7757", 2 }, /* AHA 284x BIOS disabled */
	{ "ADP7782", 3 }, /* AHA 274x Olivetti OEM */
	{ "ADP7783", 4 }, /* AHA 274x Olivetti OEM (Differential) */
	{ "ADP7770", 5 }, /* AIC7770 generic */
	{ "" }
};
MODULE_DEVICE_TABLE(eisa, aic7770_ids);

static struct eisa_driver aic7770_driver = {
	.id_table	= aic7770_ids,
	.driver = {
		.name   = "aic7xxx",
		.probe  = aic7770_probe,
		.remove = aic7770_remove,
	}
};
  
int
ahc_linux_eisa_init(void)
{
	return eisa_driver_register(&aic7770_driver);
}
  
void
ahc_linux_eisa_exit(void)
{
	eisa_driver_unregister(&aic7770_driver);
}
