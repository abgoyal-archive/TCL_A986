

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/parport.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>


#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>



#define DRVNAME		"spi-lm70llp"

#define lm70_INIT	0xBE
#define SIO		0x10
#define nCS		0x20
#define SCLK		0x40

/*-------------------------------------------------------------------------*/

struct spi_lm70llp {
	struct spi_bitbang	bitbang;
	struct parport		*port;
	struct pardevice	*pd;
	struct spi_device	*spidev_lm70;
	struct spi_board_info	info;
	//struct device		*dev;
};

/* REVISIT : ugly global ; provides "exclusive open" facility */
static struct spi_lm70llp *lm70llp;


/*-------------------------------------------------------------------*/

static inline struct spi_lm70llp *spidev_to_pp(struct spi_device *spi)
{
	return spi->controller_data;
}

/*---------------------- LM70 LLP eval board-specific inlines follow */


static inline void deassertCS(struct spi_lm70llp *pp)
{
	u8 data = parport_read_data(pp->port);

	data &= ~0x80;		/* pull D7/SI-out low while de-asserted */
	parport_write_data(pp->port, data | nCS);
}

static inline void assertCS(struct spi_lm70llp *pp)
{
	u8 data = parport_read_data(pp->port);

	data |= 0x80;		/* pull D7/SI-out high so lm70 drives SO-in */
	parport_write_data(pp->port, data & ~nCS);
}

static inline void clkHigh(struct spi_lm70llp *pp)
{
	u8 data = parport_read_data(pp->port);
	parport_write_data(pp->port, data | SCLK);
}

static inline void clkLow(struct spi_lm70llp *pp)
{
	u8 data = parport_read_data(pp->port);
	parport_write_data(pp->port, data & ~SCLK);
}

/*------------------------- SPI-LM70-specific inlines ----------------------*/

static inline void spidelay(unsigned d)
{
	udelay(d);
}

static inline void setsck(struct spi_device *s, int is_on)
{
	struct spi_lm70llp *pp = spidev_to_pp(s);

	if (is_on)
		clkHigh(pp);
	else
		clkLow(pp);
}

static inline void setmosi(struct spi_device *s, int is_on)
{
	/* FIXME update D7 ... this way we can put the chip
	 * into shutdown mode and read the manufacturer ID,
	 * but we can't put it back into operational mode.
	 */
}

static inline int getmiso(struct spi_device *s)
{
	struct spi_lm70llp *pp = spidev_to_pp(s);
	return ((SIO == (parport_read_status(pp->port) & SIO)) ? 0 : 1 );
}
/*--------------------------------------------------------------------*/

#include "spi_bitbang_txrx.h"

static void lm70_chipselect(struct spi_device *spi, int value)
{
	struct spi_lm70llp *pp = spidev_to_pp(spi);

	if (value)
		assertCS(pp);
	else
		deassertCS(pp);
}

static u32 lm70_txrx(struct spi_device *spi, unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static void spi_lm70llp_attach(struct parport *p)
{
	struct pardevice	*pd;
	struct spi_lm70llp	*pp;
	struct spi_master	*master;
	int			status;

	if (lm70llp) {
		printk(KERN_WARNING
			"%s: spi_lm70llp instance already loaded. Aborting.\n",
			DRVNAME);
		return;
	}

	/* TODO:  this just _assumes_ a lm70 is there ... no probe;
	 * the lm70 driver could verify it, reading the manf ID.
	 */

	master = spi_alloc_master(p->physport->dev, sizeof *pp);
	if (!master) {
		status = -ENOMEM;
		goto out_fail;
	}
	pp = spi_master_get_devdata(master);

	master->bus_num = -1;	/* dynamic alloc of a bus number */
	master->num_chipselect = 1;

	/*
	 * SPI and bitbang hookup.
	 */
	pp->bitbang.master = spi_master_get(master);
	pp->bitbang.chipselect = lm70_chipselect;
	pp->bitbang.txrx_word[SPI_MODE_0] = lm70_txrx;
	pp->bitbang.flags = SPI_3WIRE;

	/*
	 * Parport hookup
	 */
	pp->port = p;
	pd = parport_register_device(p, DRVNAME,
			NULL, NULL, NULL,
			PARPORT_FLAG_EXCL, pp);
	if (!pd) {
		status = -ENOMEM;
		goto out_free_master;
	}
	pp->pd = pd;

	status = parport_claim(pd);
	if (status < 0)
		goto out_parport_unreg;

	/*
	 * Start SPI ...
	 */
	status = spi_bitbang_start(&pp->bitbang);
	if (status < 0) {
		printk(KERN_WARNING
			"%s: spi_bitbang_start failed with status %d\n",
			DRVNAME, status);
		goto out_off_and_release;
	}

	/*
	 * The modalias name MUST match the device_driver name
	 * for the bus glue code to match and subsequently bind them.
	 * We are binding to the generic drivers/hwmon/lm70.c device
	 * driver.
	 */
	strcpy(pp->info.modalias, "lm70");
	pp->info.max_speed_hz = 6 * 1000 * 1000;
	pp->info.chip_select = 0;
	pp->info.mode = SPI_3WIRE | SPI_MODE_0;

	/* power up the chip, and let the LM70 control SI/SO */
	parport_write_data(pp->port, lm70_INIT);

	/* Enable access to our primary data structure via
	 * the board info's (void *)controller_data.
	 */
	pp->info.controller_data = pp;
	pp->spidev_lm70 = spi_new_device(pp->bitbang.master, &pp->info);
	if (pp->spidev_lm70)
		dev_dbg(&pp->spidev_lm70->dev, "spidev_lm70 at %s\n",
				dev_name(&pp->spidev_lm70->dev));
	else {
		printk(KERN_WARNING "%s: spi_new_device failed\n", DRVNAME);
		status = -ENODEV;
		goto out_bitbang_stop;
	}
	pp->spidev_lm70->bits_per_word = 8;

	lm70llp = pp;
	return;

out_bitbang_stop:
	spi_bitbang_stop(&pp->bitbang);
out_off_and_release:
	/* power down */
	parport_write_data(pp->port, 0);
	mdelay(10);
	parport_release(pp->pd);
out_parport_unreg:
	parport_unregister_device(pd);
out_free_master:
	(void) spi_master_put(master);
out_fail:
	pr_info("%s: spi_lm70llp probe fail, status %d\n", DRVNAME, status);
}

static void spi_lm70llp_detach(struct parport *p)
{
	struct spi_lm70llp		*pp;

	if (!lm70llp || lm70llp->port != p)
		return;

	pp = lm70llp;
	spi_bitbang_stop(&pp->bitbang);

	/* power down */
	parport_write_data(pp->port, 0);

	parport_release(pp->pd);
	parport_unregister_device(pp->pd);

	(void) spi_master_put(pp->bitbang.master);

	lm70llp = NULL;
}


static struct parport_driver spi_lm70llp_drv = {
	.name =		DRVNAME,
	.attach =	spi_lm70llp_attach,
	.detach =	spi_lm70llp_detach,
};

static int __init init_spi_lm70llp(void)
{
	return parport_register_driver(&spi_lm70llp_drv);
}
module_init(init_spi_lm70llp);

static void __exit cleanup_spi_lm70llp(void)
{
	parport_unregister_driver(&spi_lm70llp_drv);
}
module_exit(cleanup_spi_lm70llp);

MODULE_AUTHOR("Kaiwan N Billimoria <kaiwan@designergraphix.com>");
MODULE_DESCRIPTION(
	"Parport adapter for the National Semiconductor LM70 LLP eval board");
MODULE_LICENSE("GPL");
