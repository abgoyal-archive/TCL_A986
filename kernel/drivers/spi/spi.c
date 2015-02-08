

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>


static void spidev_release(struct device *dev)
{
	struct spi_device	*spi = to_spi_device(dev);

	/* spi masters may cleanup for released devices */
	if (spi->master->cleanup)
		spi->master->cleanup(spi);

	spi_master_put(spi->master);
	kfree(spi);
}

static ssize_t
modalias_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct spi_device	*spi = to_spi_device(dev);

	return sprintf(buf, "%s\n", spi->modalias);
}

static struct device_attribute spi_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};


static const struct spi_device_id *spi_match_id(const struct spi_device_id *id,
						const struct spi_device *sdev)
{
	while (id->name[0]) {
		if (!strcmp(sdev->modalias, id->name))
			return id;
		id++;
	}
	return NULL;
}

const struct spi_device_id *spi_get_device_id(const struct spi_device *sdev)
{
	const struct spi_driver *sdrv = to_spi_driver(sdev->dev.driver);

	return spi_match_id(sdrv->id_table, sdev);
}
EXPORT_SYMBOL_GPL(spi_get_device_id);

static int spi_match_device(struct device *dev, struct device_driver *drv)
{
	const struct spi_device	*spi = to_spi_device(dev);
	const struct spi_driver	*sdrv = to_spi_driver(drv);

	if (sdrv->id_table)
		return !!spi_match_id(sdrv->id_table, spi);

	return strcmp(spi->modalias, drv->name) == 0;
}

static int spi_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct spi_device		*spi = to_spi_device(dev);

	add_uevent_var(env, "MODALIAS=%s%s", SPI_MODULE_PREFIX, spi->modalias);
	return 0;
}

#ifdef	CONFIG_PM

static int spi_suspend(struct device *dev, pm_message_t message)
{
	int			value = 0;
	struct spi_driver	*drv = to_spi_driver(dev->driver);

	/* suspend will stop irqs and dma; no more i/o */
	if (drv) {
		if (drv->suspend)
			value = drv->suspend(to_spi_device(dev), message);
		else
			dev_dbg(dev, "... can't suspend\n");
	}
	return value;
}

static int spi_resume(struct device *dev)
{
	int			value = 0;
	struct spi_driver	*drv = to_spi_driver(dev->driver);

	/* resume may restart the i/o queue */
	if (drv) {
		if (drv->resume)
			value = drv->resume(to_spi_device(dev));
		else
			dev_dbg(dev, "... can't resume\n");
	}
	return value;
}

#else
#define spi_suspend	NULL
#define spi_resume	NULL
#endif

struct bus_type spi_bus_type = {
	.name		= "spi",
	.dev_attrs	= spi_dev_attrs,
	.match		= spi_match_device,
	.uevent		= spi_uevent,
	.suspend	= spi_suspend,
	.resume		= spi_resume,
};
EXPORT_SYMBOL_GPL(spi_bus_type);


static int spi_drv_probe(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	return sdrv->probe(to_spi_device(dev));
}

static int spi_drv_remove(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	return sdrv->remove(to_spi_device(dev));
}

static void spi_drv_shutdown(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	sdrv->shutdown(to_spi_device(dev));
}

int spi_register_driver(struct spi_driver *sdrv)
{
	sdrv->driver.bus = &spi_bus_type;
	if (sdrv->probe)
		sdrv->driver.probe = spi_drv_probe;
	if (sdrv->remove)
		sdrv->driver.remove = spi_drv_remove;
	if (sdrv->shutdown)
		sdrv->driver.shutdown = spi_drv_shutdown;
	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spi_register_driver);

/*-------------------------------------------------------------------------*/


struct boardinfo {
	struct list_head	list;
	unsigned		n_board_info;
	struct spi_board_info	board_info[0];
};

static LIST_HEAD(board_list);
static DEFINE_MUTEX(board_lock);

struct spi_device *spi_alloc_device(struct spi_master *master)
{
	struct spi_device	*spi;
	struct device		*dev = master->dev.parent;

	if (!spi_master_get(master))
		return NULL;

	spi = kzalloc(sizeof *spi, GFP_KERNEL);
	if (!spi) {
		dev_err(dev, "cannot alloc spi_device\n");
		spi_master_put(master);
		return NULL;
	}

	spi->master = master;
	spi->dev.parent = dev;
	spi->dev.bus = &spi_bus_type;
	spi->dev.release = spidev_release;
	device_initialize(&spi->dev);
	return spi;
}
EXPORT_SYMBOL_GPL(spi_alloc_device);

int spi_add_device(struct spi_device *spi)
{
	static DEFINE_MUTEX(spi_add_lock);
	struct device *dev = spi->master->dev.parent;
	struct device *d;
	int status;

	/* Chipselects are numbered 0..max; validate. */
	if (spi->chip_select >= spi->master->num_chipselect) {
		dev_err(dev, "cs%d >= max %d\n",
			spi->chip_select,
			spi->master->num_chipselect);
		return -EINVAL;
	}

	/* Set the bus ID string */
	dev_set_name(&spi->dev, "%s.%u", dev_name(&spi->master->dev),
			spi->chip_select);


	/* We need to make sure there's no other device with this
	 * chipselect **BEFORE** we call setup(), else we'll trash
	 * its configuration.  Lock against concurrent add() calls.
	 */
	mutex_lock(&spi_add_lock);

	d = bus_find_device_by_name(&spi_bus_type, NULL, dev_name(&spi->dev));
	if (d != NULL) {
		dev_err(dev, "chipselect %d already in use\n",
				spi->chip_select);
		put_device(d);
		status = -EBUSY;
		goto done;
	}

	/* Drivers may modify this initial i/o setup, but will
	 * normally rely on the device being setup.  Devices
	 * using SPI_CS_HIGH can't coexist well otherwise...
	 */
	status = spi_setup(spi);
	if (status < 0) {
		dev_err(dev, "can't %s %s, status %d\n",
				"setup", dev_name(&spi->dev), status);
		goto done;
	}

	/* Device may be bound to an active driver when this returns */
	status = device_add(&spi->dev);
	if (status < 0)
		dev_err(dev, "can't %s %s, status %d\n",
				"add", dev_name(&spi->dev), status);
	else
		dev_dbg(dev, "registered child %s\n", dev_name(&spi->dev));

done:
	mutex_unlock(&spi_add_lock);
	return status;
}
EXPORT_SYMBOL_GPL(spi_add_device);

struct spi_device *spi_new_device(struct spi_master *master,
				  struct spi_board_info *chip)
{
	struct spi_device	*proxy;
	int			status;

	/* NOTE:  caller did any chip->bus_num checks necessary.
	 *
	 * Also, unless we change the return value convention to use
	 * error-or-pointer (not NULL-or-pointer), troubleshootability
	 * suggests syslogged diagnostics are best here (ugh).
	 */

	proxy = spi_alloc_device(master);
	if (!proxy)
		return NULL;

	WARN_ON(strlen(chip->modalias) >= sizeof(proxy->modalias));

	proxy->chip_select = chip->chip_select;
	proxy->max_speed_hz = chip->max_speed_hz;
	proxy->mode = chip->mode;
	proxy->irq = chip->irq;
	strlcpy(proxy->modalias, chip->modalias, sizeof(proxy->modalias));
	proxy->dev.platform_data = (void *) chip->platform_data;
	proxy->controller_data = chip->controller_data;
	proxy->controller_state = NULL;

	status = spi_add_device(proxy);
	if (status < 0) {
		spi_dev_put(proxy);
		return NULL;
	}

	return proxy;
}
EXPORT_SYMBOL_GPL(spi_new_device);

int __init
spi_register_board_info(struct spi_board_info const *info, unsigned n)
{
	struct boardinfo	*bi;

	bi = kmalloc(sizeof(*bi) + n * sizeof *info, GFP_KERNEL);
	if (!bi)
		return -ENOMEM;
	bi->n_board_info = n;
	memcpy(bi->board_info, info, n * sizeof *info);

	mutex_lock(&board_lock);
	list_add_tail(&bi->list, &board_list);
	mutex_unlock(&board_lock);
	return 0;
}


static void scan_boardinfo(struct spi_master *master)
{
	struct boardinfo	*bi;

	mutex_lock(&board_lock);
	list_for_each_entry(bi, &board_list, list) {
		struct spi_board_info	*chip = bi->board_info;
		unsigned		n;

		for (n = bi->n_board_info; n > 0; n--, chip++) {
			if (chip->bus_num != master->bus_num)
				continue;
			/* NOTE: this relies on spi_new_device to
			 * issue diagnostics when given bogus inputs
			 */
			(void) spi_new_device(master, chip);
		}
	}
	mutex_unlock(&board_lock);
}

/*-------------------------------------------------------------------------*/

static void spi_master_release(struct device *dev)
{
	struct spi_master *master;

	master = container_of(dev, struct spi_master, dev);
	kfree(master);
}

static struct class spi_master_class = {
	.name		= "spi_master",
	.owner		= THIS_MODULE,
	.dev_release	= spi_master_release,
};


struct spi_master *spi_alloc_master(struct device *dev, unsigned size)
{
	struct spi_master	*master;

	if (!dev)
		return NULL;

	master = kzalloc(size + sizeof *master, GFP_KERNEL);
	if (!master)
		return NULL;

	device_initialize(&master->dev);
	master->dev.class = &spi_master_class;
	master->dev.parent = get_device(dev);
	spi_master_set_devdata(master, &master[1]);

	return master;
}
EXPORT_SYMBOL_GPL(spi_alloc_master);

int spi_register_master(struct spi_master *master)
{
	static atomic_t		dyn_bus_id = ATOMIC_INIT((1<<15) - 1);
	struct device		*dev = master->dev.parent;
	int			status = -ENODEV;
	int			dynamic = 0;

	if (!dev)
		return -ENODEV;

	/* even if it's just one always-selected device, there must
	 * be at least one chipselect
	 */
	if (master->num_chipselect == 0)
		return -EINVAL;

	/* convention:  dynamically assigned bus IDs count down from the max */
	if (master->bus_num < 0) {
		/* FIXME switch to an IDR based scheme, something like
		 * I2C now uses, so we can't run out of "dynamic" IDs
		 */
		master->bus_num = atomic_dec_return(&dyn_bus_id);
		dynamic = 1;
	}

	/* register the device, then userspace will see it.
	 * registration fails if the bus ID is in use.
	 */
	dev_set_name(&master->dev, "spi%u", master->bus_num);
	status = device_add(&master->dev);
	if (status < 0)
		goto done;
	dev_dbg(dev, "registered master %s%s\n", dev_name(&master->dev),
			dynamic ? " (dynamic)" : "");

	/* populate children from any spi device tables */
	scan_boardinfo(master);
	status = 0;
done:
	return status;
}
EXPORT_SYMBOL_GPL(spi_register_master);


static int __unregister(struct device *dev, void *master_dev)
{
	/* note: before about 2.6.14-rc1 this would corrupt memory: */
	if (dev != master_dev)
		spi_unregister_device(to_spi_device(dev));
	return 0;
}

void spi_unregister_master(struct spi_master *master)
{
	int dummy;

	dummy = device_for_each_child(master->dev.parent, &master->dev,
					__unregister);
	device_unregister(&master->dev);
}
EXPORT_SYMBOL_GPL(spi_unregister_master);

static int __spi_master_match(struct device *dev, void *data)
{
	struct spi_master *m;
	u16 *bus_num = data;

	m = container_of(dev, struct spi_master, dev);
	return m->bus_num == *bus_num;
}

struct spi_master *spi_busnum_to_master(u16 bus_num)
{
	struct device		*dev;
	struct spi_master	*master = NULL;

	dev = class_find_device(&spi_master_class, NULL, &bus_num,
				__spi_master_match);
	if (dev)
		master = container_of(dev, struct spi_master, dev);
	/* reference got in class_find_device */
	return master;
}
EXPORT_SYMBOL_GPL(spi_busnum_to_master);


/*-------------------------------------------------------------------------*/


int spi_setup(struct spi_device *spi)
{
	unsigned	bad_bits;
	int		status;

	/* help drivers fail *cleanly* when they need options
	 * that aren't supported with their current master
	 */
	bad_bits = spi->mode & ~spi->master->mode_bits;
	if (bad_bits) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			bad_bits);
		return -EINVAL;
	}

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	status = spi->master->setup(spi);

	dev_dbg(&spi->dev, "setup mode %d, %s%s%s%s"
				"%u bits/w, %u Hz max --> %d\n",
			(int) (spi->mode & (SPI_CPOL | SPI_CPHA)),
			(spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
			(spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
			(spi->mode & SPI_3WIRE) ? "3wire, " : "",
			(spi->mode & SPI_LOOP) ? "loopback, " : "",
			spi->bits_per_word, spi->max_speed_hz,
			status);

	return status;
}
EXPORT_SYMBOL_GPL(spi_setup);

int spi_async(struct spi_device *spi, struct spi_message *message)
{
	struct spi_master *master = spi->master;

	/* Half-duplex links include original MicroWire, and ones with
	 * only one data pin like SPI_3WIRE (switches direction) or where
	 * either MOSI or MISO is missing.  They can also be caused by
	 * software limitations.
	 */
	if ((master->flags & SPI_MASTER_HALF_DUPLEX)
			|| (spi->mode & SPI_3WIRE)) {
		struct spi_transfer *xfer;
		unsigned flags = master->flags;

		list_for_each_entry(xfer, &message->transfers, transfer_list) {
			if (xfer->rx_buf && xfer->tx_buf)
				return -EINVAL;
			if ((flags & SPI_MASTER_NO_TX) && xfer->tx_buf)
				return -EINVAL;
			if ((flags & SPI_MASTER_NO_RX) && xfer->rx_buf)
				return -EINVAL;
		}
	}

	message->spi = spi;
	message->status = -EINPROGRESS;
	return master->transfer(spi, message);
}
EXPORT_SYMBOL_GPL(spi_async);


/*-------------------------------------------------------------------------*/


static void spi_complete(void *arg)
{
	complete(arg);
}

int spi_sync(struct spi_device *spi, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spi_complete;
	message->context = &done;
	status = spi_async(spi, message);
	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
	}
	message->context = NULL;
	return status;
}
EXPORT_SYMBOL_GPL(spi_sync);

/* portable code must never pass more than 32 bytes */
#define	SPI_BUFSIZ	max(32,SMP_CACHE_BYTES)

static u8	*buf;

int spi_write_then_read(struct spi_device *spi,
		const u8 *txbuf, unsigned n_tx,
		u8 *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int			status;
	struct spi_message	message;
	struct spi_transfer	x[2];
	u8			*local_buf;

	/* Use preallocated DMA-safe buffer.  We can't avoid copying here,
	 * (as a pure convenience thing), but we can keep heap costs
	 * out of the hot path ...
	 */
	if ((n_tx + n_rx) > SPI_BUFSIZ)
		return -EINVAL;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if (n_tx) {
		x[0].len = n_tx;
		spi_message_add_tail(&x[0], &message);
	}
	if (n_rx) {
		x[1].len = n_rx;
		spi_message_add_tail(&x[1], &message);
	}

	/* ... unless someone else is using the pre-allocated buffer */
	if (!mutex_trylock(&lock)) {
		local_buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
		if (!local_buf)
			return -ENOMEM;
	} else
		local_buf = buf;

	memcpy(local_buf, txbuf, n_tx);
	x[0].tx_buf = local_buf;
	x[1].rx_buf = local_buf + n_tx;

	/* do the i/o */
	status = spi_sync(spi, &message);
	if (status == 0)
		memcpy(rxbuf, x[1].rx_buf, n_rx);

	if (x[0].tx_buf == buf)
		mutex_unlock(&lock);
	else
		kfree(local_buf);

	return status;
}
EXPORT_SYMBOL_GPL(spi_write_then_read);

/*-------------------------------------------------------------------------*/

static int __init spi_init(void)
{
	int	status;

	buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
	if (!buf) {
		status = -ENOMEM;
		goto err0;
	}

	status = bus_register(&spi_bus_type);
	if (status < 0)
		goto err1;

	status = class_register(&spi_master_class);
	if (status < 0)
		goto err2;
	return 0;

err2:
	bus_unregister(&spi_bus_type);
err1:
	kfree(buf);
	buf = NULL;
err0:
	return status;
}

postcore_initcall(spi_init);

