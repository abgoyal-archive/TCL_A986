

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>

#include "qib.h"


#define READ_CMD 1
#define WRITE_CMD 0

static void i2c_wait_for_writes(struct qib_devdata *dd)
{
	/*
	 * implicit read of EXTStatus is as good as explicit
	 * read of scratch, if all we want to do is flush
	 * writes.
	 */
	dd->f_gpio_mod(dd, 0, 0, 0);
	rmb(); /* inlined, so prevent compiler reordering */
}

#define SCL_WAIT_USEC 1000

#define TWSI_BUF_WAIT_USEC 60

static void scl_out(struct qib_devdata *dd, u8 bit)
{
	u32 mask;

	udelay(1);

	mask = 1UL << dd->gpio_scl_num;

	/* SCL is meant to be bare-drain, so never set "OUT", just DIR */
	dd->f_gpio_mod(dd, 0, bit ? 0 : mask, mask);

	/*
	 * Allow for slow slaves by simple
	 * delay for falling edge, sampling on rise.
	 */
	if (!bit)
		udelay(2);
	else {
		int rise_usec;
		for (rise_usec = SCL_WAIT_USEC; rise_usec > 0; rise_usec -= 2) {
			if (mask & dd->f_gpio_mod(dd, 0, 0, 0))
				break;
			udelay(2);
		}
		if (rise_usec <= 0)
			qib_dev_err(dd, "SCL interface stuck low > %d uSec\n",
				    SCL_WAIT_USEC);
	}
	i2c_wait_for_writes(dd);
}

static void sda_out(struct qib_devdata *dd, u8 bit)
{
	u32 mask;

	mask = 1UL << dd->gpio_sda_num;

	/* SDA is meant to be bare-drain, so never set "OUT", just DIR */
	dd->f_gpio_mod(dd, 0, bit ? 0 : mask, mask);

	i2c_wait_for_writes(dd);
	udelay(2);
}

static u8 sda_in(struct qib_devdata *dd, int wait)
{
	int bnum;
	u32 read_val, mask;

	bnum = dd->gpio_sda_num;
	mask = (1UL << bnum);
	/* SDA is meant to be bare-drain, so never set "OUT", just DIR */
	dd->f_gpio_mod(dd, 0, 0, mask);
	read_val = dd->f_gpio_mod(dd, 0, 0, 0);
	if (wait)
		i2c_wait_for_writes(dd);
	return (read_val & mask) >> bnum;
}

static int i2c_ackrcv(struct qib_devdata *dd)
{
	u8 ack_received;

	/* AT ENTRY SCL = LOW */
	/* change direction, ignore data */
	ack_received = sda_in(dd, 1);
	scl_out(dd, 1);
	ack_received = sda_in(dd, 1) == 0;
	scl_out(dd, 0);
	return ack_received;
}

static void stop_cmd(struct qib_devdata *dd);

static int rd_byte(struct qib_devdata *dd, int last)
{
	int bit_cntr, data;

	data = 0;

	for (bit_cntr = 7; bit_cntr >= 0; --bit_cntr) {
		data <<= 1;
		scl_out(dd, 1);
		data |= sda_in(dd, 0);
		scl_out(dd, 0);
	}
	if (last) {
		scl_out(dd, 1);
		stop_cmd(dd);
	} else {
		sda_out(dd, 0);
		scl_out(dd, 1);
		scl_out(dd, 0);
		sda_out(dd, 1);
	}
	return data;
}

static int wr_byte(struct qib_devdata *dd, u8 data)
{
	int bit_cntr;
	u8 bit;

	for (bit_cntr = 7; bit_cntr >= 0; bit_cntr--) {
		bit = (data >> bit_cntr) & 1;
		sda_out(dd, bit);
		scl_out(dd, 1);
		scl_out(dd, 0);
	}
	return (!i2c_ackrcv(dd)) ? 1 : 0;
}

static void start_seq(struct qib_devdata *dd)
{
	sda_out(dd, 1);
	scl_out(dd, 1);
	sda_out(dd, 0);
	udelay(1);
	scl_out(dd, 0);
}

static void stop_seq(struct qib_devdata *dd)
{
	scl_out(dd, 0);
	sda_out(dd, 0);
	scl_out(dd, 1);
	sda_out(dd, 1);
}

static void stop_cmd(struct qib_devdata *dd)
{
	stop_seq(dd);
	udelay(TWSI_BUF_WAIT_USEC);
}


int qib_twsi_reset(struct qib_devdata *dd)
{
	int clock_cycles_left = 9;
	int was_high = 0;
	u32 pins, mask;

	/* Both SCL and SDA should be high. If not, there
	 * is something wrong.
	 */
	mask = (1UL << dd->gpio_scl_num) | (1UL << dd->gpio_sda_num);

	/*
	 * Force pins to desired innocuous state.
	 * This is the default power-on state with out=0 and dir=0,
	 * So tri-stated and should be floating high (barring HW problems)
	 */
	dd->f_gpio_mod(dd, 0, 0, mask);

	/*
	 * Clock nine times to get all listeners into a sane state.
	 * If SDA does not go high at any point, we are wedged.
	 * One vendor recommends then issuing START followed by STOP.
	 * we cannot use our "normal" functions to do that, because
	 * if SCL drops between them, another vendor's part will
	 * wedge, dropping SDA and keeping it low forever, at the end of
	 * the next transaction (even if it was not the device addressed).
	 * So our START and STOP take place with SCL held high.
	 */
	while (clock_cycles_left--) {
		scl_out(dd, 0);
		scl_out(dd, 1);
		/* Note if SDA is high, but keep clocking to sync slave */
		was_high |= sda_in(dd, 0);
	}

	if (was_high) {
		/*
		 * We saw a high, which we hope means the slave is sync'd.
		 * Issue START, STOP, pause for T_BUF.
		 */

		pins = dd->f_gpio_mod(dd, 0, 0, 0);
		if ((pins & mask) != mask)
			qib_dev_err(dd, "GPIO pins not at rest: %d\n",
				    pins & mask);
		/* Drop SDA to issue START */
		udelay(1); /* Guarantee .6 uSec setup */
		sda_out(dd, 0);
		udelay(1); /* Guarantee .6 uSec hold */
		/* At this point, SCL is high, SDA low. Raise SDA for STOP */
		sda_out(dd, 1);
		udelay(TWSI_BUF_WAIT_USEC);
	}

	return !was_high;
}

#define QIB_TWSI_START 0x100
#define QIB_TWSI_STOP 0x200

static int qib_twsi_wr(struct qib_devdata *dd, int data, int flags)
{
	int ret = 1;
	if (flags & QIB_TWSI_START)
		start_seq(dd);

	ret = wr_byte(dd, data); /* Leaves SCL low (from i2c_ackrcv()) */

	if (flags & QIB_TWSI_STOP)
		stop_cmd(dd);
	return ret;
}

/* Added functionality for IBA7220-based cards */
#define QIB_TEMP_DEV 0x98

int qib_twsi_blk_rd(struct qib_devdata *dd, int dev, int addr,
		    void *buffer, int len)
{
	int ret;
	u8 *bp = buffer;

	ret = 1;

	if (dev == QIB_TWSI_NO_DEV) {
		/* legacy not-really-I2C */
		addr = (addr << 1) | READ_CMD;
		ret = qib_twsi_wr(dd, addr, QIB_TWSI_START);
	} else {
		/* Actual I2C */
		ret = qib_twsi_wr(dd, dev | WRITE_CMD, QIB_TWSI_START);
		if (ret) {
			stop_cmd(dd);
			ret = 1;
			goto bail;
		}
		/*
		 * SFF spec claims we do _not_ stop after the addr
		 * but simply issue a start with the "read" dev-addr.
		 * Since we are implicitely waiting for ACK here,
		 * we need t_buf (nominally 20uSec) before that start,
		 * and cannot rely on the delay built in to the STOP
		 */
		ret = qib_twsi_wr(dd, addr, 0);
		udelay(TWSI_BUF_WAIT_USEC);

		if (ret) {
			qib_dev_err(dd,
				"Failed to write interface read addr %02X\n",
				addr);
			ret = 1;
			goto bail;
		}
		ret = qib_twsi_wr(dd, dev | READ_CMD, QIB_TWSI_START);
	}
	if (ret) {
		stop_cmd(dd);
		ret = 1;
		goto bail;
	}

	/*
	 * block devices keeps clocking data out as long as we ack,
	 * automatically incrementing the address. Some have "pages"
	 * whose boundaries will not be crossed, but the handling
	 * of these is left to the caller, who is in a better
	 * position to know.
	 */
	while (len-- > 0) {
		/*
		 * Get and store data, sending ACK if length remaining,
		 * else STOP
		 */
		*bp++ = rd_byte(dd, !len);
	}

	ret = 0;

bail:
	return ret;
}

int qib_twsi_blk_wr(struct qib_devdata *dd, int dev, int addr,
		    const void *buffer, int len)
{
	int sub_len;
	const u8 *bp = buffer;
	int max_wait_time, i;
	int ret;
	ret = 1;

	while (len > 0) {
		if (dev == QIB_TWSI_NO_DEV) {
			if (qib_twsi_wr(dd, (addr << 1) | WRITE_CMD,
					QIB_TWSI_START)) {
				goto failed_write;
			}
		} else {
			/* Real I2C */
			if (qib_twsi_wr(dd, dev | WRITE_CMD, QIB_TWSI_START))
				goto failed_write;
			ret = qib_twsi_wr(dd, addr, 0);
			if (ret) {
				qib_dev_err(dd, "Failed to write interface"
					    " write addr %02X\n", addr);
				goto failed_write;
			}
		}

		sub_len = min(len, 4);
		addr += sub_len;
		len -= sub_len;

		for (i = 0; i < sub_len; i++)
			if (qib_twsi_wr(dd, *bp++, 0))
				goto failed_write;

		stop_cmd(dd);

		/*
		 * Wait for write complete by waiting for a successful
		 * read (the chip replies with a zero after the write
		 * cmd completes, and before it writes to the eeprom.
		 * The startcmd for the read will fail the ack until
		 * the writes have completed.   We do this inline to avoid
		 * the debug prints that are in the real read routine
		 * if the startcmd fails.
		 * We also use the proper device address, so it doesn't matter
		 * whether we have real eeprom_dev. Legacy likes any address.
		 */
		max_wait_time = 100;
		while (qib_twsi_wr(dd, dev | READ_CMD, QIB_TWSI_START)) {
			stop_cmd(dd);
			if (!--max_wait_time)
				goto failed_write;
		}
		/* now read (and ignore) the resulting byte */
		rd_byte(dd, 1);
	}

	ret = 0;
	goto bail;

failed_write:
	stop_cmd(dd);
	ret = 1;

bail:
	return ret;
}
