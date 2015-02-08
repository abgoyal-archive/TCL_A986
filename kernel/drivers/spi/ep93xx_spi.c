

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>

#include <mach/ep93xx_spi.h>

#define SSPCR0			0x0000
#define SSPCR0_MODE_SHIFT	6
#define SSPCR0_SCR_SHIFT	8

#define SSPCR1			0x0004
#define SSPCR1_RIE		BIT(0)
#define SSPCR1_TIE		BIT(1)
#define SSPCR1_RORIE		BIT(2)
#define SSPCR1_LBM		BIT(3)
#define SSPCR1_SSE		BIT(4)
#define SSPCR1_MS		BIT(5)
#define SSPCR1_SOD		BIT(6)

#define SSPDR			0x0008

#define SSPSR			0x000c
#define SSPSR_TFE		BIT(0)
#define SSPSR_TNF		BIT(1)
#define SSPSR_RNE		BIT(2)
#define SSPSR_RFF		BIT(3)
#define SSPSR_BSY		BIT(4)
#define SSPCPSR			0x0010

#define SSPIIR			0x0014
#define SSPIIR_RIS		BIT(0)
#define SSPIIR_TIS		BIT(1)
#define SSPIIR_RORIS		BIT(2)
#define SSPICR			SSPIIR

/* timeout in milliseconds */
#define SPI_TIMEOUT		5
/* maximum depth of RX/TX FIFO */
#define SPI_FIFO_SIZE		8

struct ep93xx_spi {
	spinlock_t			lock;
	const struct platform_device	*pdev;
	struct clk			*clk;
	void __iomem			*regs_base;
	int				irq;
	unsigned long			min_rate;
	unsigned long			max_rate;
	bool				running;
	struct workqueue_struct		*wq;
	struct work_struct		msg_work;
	struct completion		wait;
	struct list_head		msg_queue;
	struct spi_message		*current_msg;
	size_t				tx;
	size_t				rx;
	size_t				fifo_level;
};

struct ep93xx_spi_chip {
	const struct spi_device		*spi;
	unsigned long			rate;
	u8				div_cpsr;
	u8				div_scr;
	u8				dss;
	struct ep93xx_spi_chip_ops	*ops;
};

/* converts bits per word to CR0.DSS value */
#define bits_per_word_to_dss(bpw)	((bpw) - 1)

static inline void
ep93xx_spi_write_u8(const struct ep93xx_spi *espi, u16 reg, u8 value)
{
	__raw_writeb(value, espi->regs_base + reg);
}

static inline u8
ep93xx_spi_read_u8(const struct ep93xx_spi *spi, u16 reg)
{
	return __raw_readb(spi->regs_base + reg);
}

static inline void
ep93xx_spi_write_u16(const struct ep93xx_spi *espi, u16 reg, u16 value)
{
	__raw_writew(value, espi->regs_base + reg);
}

static inline u16
ep93xx_spi_read_u16(const struct ep93xx_spi *spi, u16 reg)
{
	return __raw_readw(spi->regs_base + reg);
}

static int ep93xx_spi_enable(const struct ep93xx_spi *espi)
{
	u8 regval;
	int err;

	err = clk_enable(espi->clk);
	if (err)
		return err;

	regval = ep93xx_spi_read_u8(espi, SSPCR1);
	regval |= SSPCR1_SSE;
	ep93xx_spi_write_u8(espi, SSPCR1, regval);

	return 0;
}

static void ep93xx_spi_disable(const struct ep93xx_spi *espi)
{
	u8 regval;

	regval = ep93xx_spi_read_u8(espi, SSPCR1);
	regval &= ~SSPCR1_SSE;
	ep93xx_spi_write_u8(espi, SSPCR1, regval);

	clk_disable(espi->clk);
}

static void ep93xx_spi_enable_interrupts(const struct ep93xx_spi *espi)
{
	u8 regval;

	regval = ep93xx_spi_read_u8(espi, SSPCR1);
	regval |= (SSPCR1_RORIE | SSPCR1_TIE | SSPCR1_RIE);
	ep93xx_spi_write_u8(espi, SSPCR1, regval);
}

static void ep93xx_spi_disable_interrupts(const struct ep93xx_spi *espi)
{
	u8 regval;

	regval = ep93xx_spi_read_u8(espi, SSPCR1);
	regval &= ~(SSPCR1_RORIE | SSPCR1_TIE | SSPCR1_RIE);
	ep93xx_spi_write_u8(espi, SSPCR1, regval);
}

static int ep93xx_spi_calc_divisors(const struct ep93xx_spi *espi,
				    struct ep93xx_spi_chip *chip,
				    unsigned long rate)
{
	unsigned long spi_clk_rate = clk_get_rate(espi->clk);
	int cpsr, scr;

	/*
	 * Make sure that max value is between values supported by the
	 * controller. Note that minimum value is already checked in
	 * ep93xx_spi_transfer().
	 */
	rate = clamp(rate, espi->min_rate, espi->max_rate);

	/*
	 * Calculate divisors so that we can get speed according the
	 * following formula:
	 *	rate = spi_clock_rate / (cpsr * (1 + scr))
	 *
	 * cpsr must be even number and starts from 2, scr can be any number
	 * between 0 and 255.
	 */
	for (cpsr = 2; cpsr <= 254; cpsr += 2) {
		for (scr = 0; scr <= 255; scr++) {
			if ((spi_clk_rate / (cpsr * (scr + 1))) <= rate) {
				chip->div_scr = (u8)scr;
				chip->div_cpsr = (u8)cpsr;
				return 0;
			}
		}
	}

	return -EINVAL;
}

static void ep93xx_spi_cs_control(struct spi_device *spi, bool control)
{
	struct ep93xx_spi_chip *chip = spi_get_ctldata(spi);
	int value = (spi->mode & SPI_CS_HIGH) ? control : !control;

	if (chip->ops && chip->ops->cs_control)
		chip->ops->cs_control(spi, value);
}

static int ep93xx_spi_setup(struct spi_device *spi)
{
	struct ep93xx_spi *espi = spi_master_get_devdata(spi->master);
	struct ep93xx_spi_chip *chip;

	if (spi->bits_per_word < 4 || spi->bits_per_word > 16) {
		dev_err(&espi->pdev->dev, "invalid bits per word %d\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	chip = spi_get_ctldata(spi);
	if (!chip) {
		dev_dbg(&espi->pdev->dev, "initial setup for %s\n",
			spi->modalias);

		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		chip->spi = spi;
		chip->ops = spi->controller_data;

		if (chip->ops && chip->ops->setup) {
			int ret = chip->ops->setup(spi);
			if (ret) {
				kfree(chip);
				return ret;
			}
		}

		spi_set_ctldata(spi, chip);
	}

	if (spi->max_speed_hz != chip->rate) {
		int err;

		err = ep93xx_spi_calc_divisors(espi, chip, spi->max_speed_hz);
		if (err != 0) {
			spi_set_ctldata(spi, NULL);
			kfree(chip);
			return err;
		}
		chip->rate = spi->max_speed_hz;
	}

	chip->dss = bits_per_word_to_dss(spi->bits_per_word);

	ep93xx_spi_cs_control(spi, false);
	return 0;
}

static int ep93xx_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct ep93xx_spi *espi = spi_master_get_devdata(spi->master);
	struct spi_transfer *t;
	unsigned long flags;

	if (!msg || !msg->complete)
		return -EINVAL;

	/* first validate each transfer */
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		if (t->bits_per_word) {
			if (t->bits_per_word < 4 || t->bits_per_word > 16)
				return -EINVAL;
		}
		if (t->speed_hz && t->speed_hz < espi->min_rate)
				return -EINVAL;
	}

	/*
	 * Now that we own the message, let's initialize it so that it is
	 * suitable for us. We use @msg->status to signal whether there was
	 * error in transfer and @msg->state is used to hold pointer to the
	 * current transfer (or %NULL if no active current transfer).
	 */
	msg->state = NULL;
	msg->status = 0;
	msg->actual_length = 0;

	spin_lock_irqsave(&espi->lock, flags);
	if (!espi->running) {
		spin_unlock_irqrestore(&espi->lock, flags);
		return -ESHUTDOWN;
	}
	list_add_tail(&msg->queue, &espi->msg_queue);
	queue_work(espi->wq, &espi->msg_work);
	spin_unlock_irqrestore(&espi->lock, flags);

	return 0;
}

static void ep93xx_spi_cleanup(struct spi_device *spi)
{
	struct ep93xx_spi_chip *chip;

	chip = spi_get_ctldata(spi);
	if (chip) {
		if (chip->ops && chip->ops->cleanup)
			chip->ops->cleanup(spi);
		spi_set_ctldata(spi, NULL);
		kfree(chip);
	}
}

static void ep93xx_spi_chip_setup(const struct ep93xx_spi *espi,
				  const struct ep93xx_spi_chip *chip)
{
	u16 cr0;

	cr0 = chip->div_scr << SSPCR0_SCR_SHIFT;
	cr0 |= (chip->spi->mode & (SPI_CPHA|SPI_CPOL)) << SSPCR0_MODE_SHIFT;
	cr0 |= chip->dss;

	dev_dbg(&espi->pdev->dev, "setup: mode %d, cpsr %d, scr %d, dss %d\n",
		chip->spi->mode, chip->div_cpsr, chip->div_scr, chip->dss);
	dev_dbg(&espi->pdev->dev, "setup: cr0 %#x", cr0);

	ep93xx_spi_write_u8(espi, SSPCPSR, chip->div_cpsr);
	ep93xx_spi_write_u16(espi, SSPCR0, cr0);
}

static inline int bits_per_word(const struct ep93xx_spi *espi)
{
	struct spi_message *msg = espi->current_msg;
	struct spi_transfer *t = msg->state;

	return t->bits_per_word ? t->bits_per_word : msg->spi->bits_per_word;
}

static void ep93xx_do_write(struct ep93xx_spi *espi, struct spi_transfer *t)
{
	if (bits_per_word(espi) > 8) {
		u16 tx_val = 0;

		if (t->tx_buf)
			tx_val = ((u16 *)t->tx_buf)[espi->tx];
		ep93xx_spi_write_u16(espi, SSPDR, tx_val);
		espi->tx += sizeof(tx_val);
	} else {
		u8 tx_val = 0;

		if (t->tx_buf)
			tx_val = ((u8 *)t->tx_buf)[espi->tx];
		ep93xx_spi_write_u8(espi, SSPDR, tx_val);
		espi->tx += sizeof(tx_val);
	}
}

static void ep93xx_do_read(struct ep93xx_spi *espi, struct spi_transfer *t)
{
	if (bits_per_word(espi) > 8) {
		u16 rx_val;

		rx_val = ep93xx_spi_read_u16(espi, SSPDR);
		if (t->rx_buf)
			((u16 *)t->rx_buf)[espi->rx] = rx_val;
		espi->rx += sizeof(rx_val);
	} else {
		u8 rx_val;

		rx_val = ep93xx_spi_read_u8(espi, SSPDR);
		if (t->rx_buf)
			((u8 *)t->rx_buf)[espi->rx] = rx_val;
		espi->rx += sizeof(rx_val);
	}
}

static int ep93xx_spi_read_write(struct ep93xx_spi *espi)
{
	struct spi_message *msg = espi->current_msg;
	struct spi_transfer *t = msg->state;

	/* read as long as RX FIFO has frames in it */
	while ((ep93xx_spi_read_u8(espi, SSPSR) & SSPSR_RNE)) {
		ep93xx_do_read(espi, t);
		espi->fifo_level--;
	}

	/* write as long as TX FIFO has room */
	while (espi->fifo_level < SPI_FIFO_SIZE && espi->tx < t->len) {
		ep93xx_do_write(espi, t);
		espi->fifo_level++;
	}

	if (espi->rx == t->len) {
		msg->actual_length += t->len;
		return 0;
	}

	return -EINPROGRESS;
}

static void ep93xx_spi_process_transfer(struct ep93xx_spi *espi,
					struct spi_message *msg,
					struct spi_transfer *t)
{
	struct ep93xx_spi_chip *chip = spi_get_ctldata(msg->spi);

	msg->state = t;

	/*
	 * Handle any transfer specific settings if needed. We use
	 * temporary chip settings here and restore original later when
	 * the transfer is finished.
	 */
	if (t->speed_hz || t->bits_per_word) {
		struct ep93xx_spi_chip tmp_chip = *chip;

		if (t->speed_hz) {
			int err;

			err = ep93xx_spi_calc_divisors(espi, &tmp_chip,
						       t->speed_hz);
			if (err) {
				dev_err(&espi->pdev->dev,
					"failed to adjust speed\n");
				msg->status = err;
				return;
			}
		}

		if (t->bits_per_word)
			tmp_chip.dss = bits_per_word_to_dss(t->bits_per_word);

		/*
		 * Set up temporary new hw settings for this transfer.
		 */
		ep93xx_spi_chip_setup(espi, &tmp_chip);
	}

	espi->rx = 0;
	espi->tx = 0;

	/*
	 * Now everything is set up for the current transfer. We prime the TX
	 * FIFO, enable interrupts, and wait for the transfer to complete.
	 */
	if (ep93xx_spi_read_write(espi)) {
		ep93xx_spi_enable_interrupts(espi);
		wait_for_completion(&espi->wait);
	}

	/*
	 * In case of error during transmit, we bail out from processing
	 * the message.
	 */
	if (msg->status)
		return;

	/*
	 * After this transfer is finished, perform any possible
	 * post-transfer actions requested by the protocol driver.
	 */
	if (t->delay_usecs) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(usecs_to_jiffies(t->delay_usecs));
	}
	if (t->cs_change) {
		if (!list_is_last(&t->transfer_list, &msg->transfers)) {
			/*
			 * In case protocol driver is asking us to drop the
			 * chipselect briefly, we let the scheduler to handle
			 * any "delay" here.
			 */
			ep93xx_spi_cs_control(msg->spi, false);
			cond_resched();
			ep93xx_spi_cs_control(msg->spi, true);
		}
	}

	if (t->speed_hz || t->bits_per_word)
		ep93xx_spi_chip_setup(espi, chip);
}

static void ep93xx_spi_process_message(struct ep93xx_spi *espi,
				       struct spi_message *msg)
{
	unsigned long timeout;
	struct spi_transfer *t;
	int err;

	/*
	 * Enable the SPI controller and its clock.
	 */
	err = ep93xx_spi_enable(espi);
	if (err) {
		dev_err(&espi->pdev->dev, "failed to enable SPI controller\n");
		msg->status = err;
		return;
	}

	/*
	 * Just to be sure: flush any data from RX FIFO.
	 */
	timeout = jiffies + msecs_to_jiffies(SPI_TIMEOUT);
	while (ep93xx_spi_read_u16(espi, SSPSR) & SSPSR_RNE) {
		if (time_after(jiffies, timeout)) {
			dev_warn(&espi->pdev->dev,
				 "timeout while flushing RX FIFO\n");
			msg->status = -ETIMEDOUT;
			return;
		}
		ep93xx_spi_read_u16(espi, SSPDR);
	}

	/*
	 * We explicitly handle FIFO level. This way we don't have to check TX
	 * FIFO status using %SSPSR_TNF bit which may cause RX FIFO overruns.
	 */
	espi->fifo_level = 0;

	/*
	 * Update SPI controller registers according to spi device and assert
	 * the chipselect.
	 */
	ep93xx_spi_chip_setup(espi, spi_get_ctldata(msg->spi));
	ep93xx_spi_cs_control(msg->spi, true);

	list_for_each_entry(t, &msg->transfers, transfer_list) {
		ep93xx_spi_process_transfer(espi, msg, t);
		if (msg->status)
			break;
	}

	/*
	 * Now the whole message is transferred (or failed for some reason). We
	 * deselect the device and disable the SPI controller.
	 */
	ep93xx_spi_cs_control(msg->spi, false);
	ep93xx_spi_disable(espi);
}

#define work_to_espi(work) (container_of((work), struct ep93xx_spi, msg_work))

static void ep93xx_spi_work(struct work_struct *work)
{
	struct ep93xx_spi *espi = work_to_espi(work);
	struct spi_message *msg;

	spin_lock_irq(&espi->lock);
	if (!espi->running || espi->current_msg ||
		list_empty(&espi->msg_queue)) {
		spin_unlock_irq(&espi->lock);
		return;
	}
	msg = list_first_entry(&espi->msg_queue, struct spi_message, queue);
	list_del_init(&msg->queue);
	espi->current_msg = msg;
	spin_unlock_irq(&espi->lock);

	ep93xx_spi_process_message(espi, msg);

	/*
	 * Update the current message and re-schedule ourselves if there are
	 * more messages in the queue.
	 */
	spin_lock_irq(&espi->lock);
	espi->current_msg = NULL;
	if (espi->running && !list_empty(&espi->msg_queue))
		queue_work(espi->wq, &espi->msg_work);
	spin_unlock_irq(&espi->lock);

	/* notify the protocol driver that we are done with this message */
	msg->complete(msg->context);
}

static irqreturn_t ep93xx_spi_interrupt(int irq, void *dev_id)
{
	struct ep93xx_spi *espi = dev_id;
	u8 irq_status = ep93xx_spi_read_u8(espi, SSPIIR);

	/*
	 * If we got ROR (receive overrun) interrupt we know that something is
	 * wrong. Just abort the message.
	 */
	if (unlikely(irq_status & SSPIIR_RORIS)) {
		/* clear the overrun interrupt */
		ep93xx_spi_write_u8(espi, SSPICR, 0);
		dev_warn(&espi->pdev->dev,
			 "receive overrun, aborting the message\n");
		espi->current_msg->status = -EIO;
	} else {
		/*
		 * Interrupt is either RX (RIS) or TX (TIS). For both cases we
		 * simply execute next data transfer.
		 */
		if (ep93xx_spi_read_write(espi)) {
			/*
			 * In normal case, there still is some processing left
			 * for current transfer. Let's wait for the next
			 * interrupt then.
			 */
			return IRQ_HANDLED;
		}
	}

	/*
	 * Current transfer is finished, either with error or with success. In
	 * any case we disable interrupts and notify the worker to handle
	 * any post-processing of the message.
	 */
	ep93xx_spi_disable_interrupts(espi);
	complete(&espi->wait);
	return IRQ_HANDLED;
}

static int __init ep93xx_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct ep93xx_spi_info *info;
	struct ep93xx_spi *espi;
	struct resource *res;
	int error;

	info = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof(*espi));
	if (!master) {
		dev_err(&pdev->dev, "failed to allocate spi master\n");
		return -ENOMEM;
	}

	master->setup = ep93xx_spi_setup;
	master->transfer = ep93xx_spi_transfer;
	master->cleanup = ep93xx_spi_cleanup;
	master->bus_num = pdev->id;
	master->num_chipselect = info->num_chipselect;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	platform_set_drvdata(pdev, master);

	espi = spi_master_get_devdata(master);

	espi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(espi->clk)) {
		dev_err(&pdev->dev, "unable to get spi clock\n");
		error = PTR_ERR(espi->clk);
		goto fail_release_master;
	}

	spin_lock_init(&espi->lock);
	init_completion(&espi->wait);

	/*
	 * Calculate maximum and minimum supported clock rates
	 * for the controller.
	 */
	espi->max_rate = clk_get_rate(espi->clk) / 2;
	espi->min_rate = clk_get_rate(espi->clk) / (254 * 256);
	espi->pdev = pdev;

	espi->irq = platform_get_irq(pdev, 0);
	if (espi->irq < 0) {
		error = -EBUSY;
		dev_err(&pdev->dev, "failed to get irq resources\n");
		goto fail_put_clock;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "unable to get iomem resource\n");
		error = -ENODEV;
		goto fail_put_clock;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "unable to request iomem resources\n");
		error = -EBUSY;
		goto fail_put_clock;
	}

	espi->regs_base = ioremap(res->start, resource_size(res));
	if (!espi->regs_base) {
		dev_err(&pdev->dev, "failed to map resources\n");
		error = -ENODEV;
		goto fail_free_mem;
	}

	error = request_irq(espi->irq, ep93xx_spi_interrupt, 0,
			    "ep93xx-spi", espi);
	if (error) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto fail_unmap_regs;
	}

	espi->wq = create_singlethread_workqueue("ep93xx_spid");
	if (!espi->wq) {
		dev_err(&pdev->dev, "unable to create workqueue\n");
		goto fail_free_irq;
	}
	INIT_WORK(&espi->msg_work, ep93xx_spi_work);
	INIT_LIST_HEAD(&espi->msg_queue);
	espi->running = true;

	/* make sure that the hardware is disabled */
	ep93xx_spi_write_u8(espi, SSPCR1, 0);

	error = spi_register_master(master);
	if (error) {
		dev_err(&pdev->dev, "failed to register SPI master\n");
		goto fail_free_queue;
	}

	dev_info(&pdev->dev, "EP93xx SPI Controller at 0x%08lx irq %d\n",
		 (unsigned long)res->start, espi->irq);

	return 0;

fail_free_queue:
	destroy_workqueue(espi->wq);
fail_free_irq:
	free_irq(espi->irq, espi);
fail_unmap_regs:
	iounmap(espi->regs_base);
fail_free_mem:
	release_mem_region(res->start, resource_size(res));
fail_put_clock:
	clk_put(espi->clk);
fail_release_master:
	spi_master_put(master);
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int __exit ep93xx_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct ep93xx_spi *espi = spi_master_get_devdata(master);
	struct resource *res;

	spin_lock_irq(&espi->lock);
	espi->running = false;
	spin_unlock_irq(&espi->lock);

	destroy_workqueue(espi->wq);

	/*
	 * Complete remaining messages with %-ESHUTDOWN status.
	 */
	spin_lock_irq(&espi->lock);
	while (!list_empty(&espi->msg_queue)) {
		struct spi_message *msg;

		msg = list_first_entry(&espi->msg_queue,
				       struct spi_message, queue);
		list_del_init(&msg->queue);
		msg->status = -ESHUTDOWN;
		spin_unlock_irq(&espi->lock);
		msg->complete(msg->context);
		spin_lock_irq(&espi->lock);
	}
	spin_unlock_irq(&espi->lock);

	free_irq(espi->irq, espi);
	iounmap(espi->regs_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	clk_put(espi->clk);
	platform_set_drvdata(pdev, NULL);

	spi_unregister_master(master);
	return 0;
}

static struct platform_driver ep93xx_spi_driver = {
	.driver		= {
		.name	= "ep93xx-spi",
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(ep93xx_spi_remove),
};

static int __init ep93xx_spi_init(void)
{
	return platform_driver_probe(&ep93xx_spi_driver, ep93xx_spi_probe);
}
module_init(ep93xx_spi_init);

static void __exit ep93xx_spi_exit(void)
{
	platform_driver_unregister(&ep93xx_spi_driver);
}
module_exit(ep93xx_spi_exit);

MODULE_DESCRIPTION("EP93xx SPI Controller driver");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@iki.fi>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ep93xx-spi");
