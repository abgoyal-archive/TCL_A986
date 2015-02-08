
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/pm.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/setup.h>
#include <asm/intel_scu_ipc.h>

/* IPC defines the following message types */
#define IPCMSG_WATCHDOG_TIMER 0xF8 /* Set Kernel Watchdog Threshold */
#define IPCMSG_BATTERY        0xEF /* Coulomb Counter Accumulator */
#define IPCMSG_FW_UPDATE      0xFE /* Firmware update */
#define IPCMSG_PCNTRL         0xFF /* Power controller unit read/write */
#define IPCMSG_FW_REVISION    0xF4 /* Get firmware revision */

/* Command id associated with message IPCMSG_PCNTRL */
#define IPC_CMD_PCNTRL_W      0 /* Register write */
#define IPC_CMD_PCNTRL_R      1 /* Register read */
#define IPC_CMD_PCNTRL_M      2 /* Register read-modify-write */

/* Miscelaneous Command ids */
#define IPC_CMD_INDIRECT_RD   2 /* 32bit indirect read */
#define IPC_CMD_INDIRECT_WR   5 /* 32bit indirect write */


#define IPC_BASE_ADDR     0xFF11C000	/* IPC1 base register address */
#define IPC_MAX_ADDR      0x100		/* Maximum IPC regisers */
#define IPC_WWBUF_SIZE    16		/* IPC Write buffer Size */
#define IPC_RWBUF_SIZE    16		/* IPC Read buffer Size */
#define IPC_I2C_BASE      0xFF12B000	/* I2C control register base address */
#define IPC_I2C_MAX_ADDR  0x10		/* Maximum I2C regisers */

static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void ipc_remove(struct pci_dev *pdev);

struct intel_scu_ipc_dev {
	struct pci_dev *pdev;
	void __iomem *ipc_base;
	void __iomem *i2c_base;
};

static struct intel_scu_ipc_dev  ipcdev; /* Only one for now */

static int platform = 1;
module_param(platform, int, 0);
MODULE_PARM_DESC(platform, "1 for moorestown platform");




#define IPC_READ_BUFFER		0x90

#define IPC_I2C_CNTRL_ADDR	0
#define I2C_DATA_ADDR		0x04

static DEFINE_MUTEX(ipclock); /* lock used to prevent multiple call to SCU */

static inline void ipc_command(u32 cmd) /* Send ipc command */
{
	writel(cmd, ipcdev.ipc_base);
}

static inline void ipc_data_writel(u32 data, u32 offset) /* Write ipc data */
{
	writel(data, ipcdev.ipc_base + 0x80 + offset);
}

static inline void ipc_write_dptr(u32 data) /* Write dptr data */
{
	writel(data, ipcdev.ipc_base + 0x0C);
}

static inline void ipc_write_sptr(u32 data) /* Write dptr data */
{
	writel(data, ipcdev.ipc_base + 0x08);
}


static inline u8 ipc_read_status(void)
{
	return __raw_readl(ipcdev.ipc_base + 0x04);
}

static inline u8 ipc_data_readb(u32 offset) /* Read ipc byte data */
{
	return readb(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

static inline u8 ipc_data_readl(u32 offset) /* Read ipc u32 data */
{
	return readl(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

static inline int busy_loop(void) /* Wait till scu status is busy */
{
	u32 status = 0;
	u32 loop_count = 0;

	status = ipc_read_status();
	while (status & 1) {
		udelay(1); /* scu processing time is in few u secods */
		status = ipc_read_status();
		loop_count++;
		/* break if scu doesn't reset busy bit after huge retry */
		if (loop_count > 100000) {
			dev_err(&ipcdev.pdev->dev, "IPC timed out");
			return -ETIMEDOUT;
		}
	}
	return (status >> 1) & 1;
}

/* Read/Write power control(PMIC in Langwell, MSIC in PenWell) registers */
static int pwr_reg_rdwr(u16 *addr, u8 *data, u32 count, u32 op, u32 id)
{
	int nc;
	u32 offset = 0;
	u32 err = 0;
	u8 cbuf[IPC_WWBUF_SIZE] = { '\0' };
	u32 *wbuf = (u32 *)&cbuf;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	if (platform == 1) {
		/* Entry is 4 bytes for read/write, 5 bytes for read modify */
		for (nc = 0; nc < count; nc++) {
			cbuf[offset] = addr[nc];
			cbuf[offset + 1] = addr[nc] >> 8;
			if (id != IPC_CMD_PCNTRL_R)
				cbuf[offset + 2] = data[nc];
			if (id == IPC_CMD_PCNTRL_M) {
				cbuf[offset + 3] = data[nc + 1];
				offset += 1;
			}
			offset += 3;
		}
		for (nc = 0, offset = 0; nc < count; nc++, offset += 4)
			ipc_data_writel(wbuf[nc], offset); /* Write wbuff */

	} else {
		for (nc = 0, offset = 0; nc < count; nc++, offset += 2)
			ipc_data_writel(addr[nc], offset); /* Write addresses */
		if (id != IPC_CMD_PCNTRL_R) {
			for (nc = 0; nc < count; nc++, offset++)
				ipc_data_writel(data[nc], offset); /* Write data */
			if (id == IPC_CMD_PCNTRL_M)
				ipc_data_writel(data[nc + 1], offset); /* Mask value*/
		}
	}

	if (id != IPC_CMD_PCNTRL_M)
		ipc_command((count * 3) << 16 |  id << 12 | 0 << 8 | op);
	else
		ipc_command((count * 4) << 16 |  id << 12 | 0 << 8 | op);

	err = busy_loop();

	if (id == IPC_CMD_PCNTRL_R) { /* Read rbuf */
		/* Workaround: values are read as 0 without memcpy_fromio */
		memcpy_fromio(cbuf, ipcdev.ipc_base + IPC_READ_BUFFER, 16);
		if (platform == 1) {
			for (nc = 0, offset = 2; nc < count; nc++, offset += 3)
				data[nc] = ipc_data_readb(offset);
		} else {
			for (nc = 0; nc < count; nc++)
				data[nc] = ipc_data_readb(nc);
		}
	}
	mutex_unlock(&ipclock);
	return err;
}

int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

int intel_scu_ipc_ioread16(u16 addr, u16 *data)
{
	u16 x[2] = {addr, addr + 1 };
	return pwr_reg_rdwr(x, (u8 *)data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread16);

int intel_scu_ipc_ioread32(u16 addr, u32 *data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread32);

int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return pwr_reg_rdwr(&addr, &data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

int intel_scu_ipc_iowrite16(u16 addr, u16 data)
{
	u16 x[2] = {addr, addr + 1 };
	return pwr_reg_rdwr(x, (u8 *)&data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite16);

int intel_scu_ipc_iowrite32(u16 addr, u32 data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)&data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite32);

int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_writev);


int intel_scu_ipc_update_register(u16 addr, u8 bits, u8 mask)
{
	u8 data[2] = { bits, mask };
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_M);
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

int intel_scu_ipc_register_read(u32 addr, u32 *value)
{
	u32 err = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	ipc_write_sptr(addr);
	ipc_command(4 << 16 | IPC_CMD_INDIRECT_RD);
	err = busy_loop();
	*value = ipc_data_readl(0);
	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_register_read);

int intel_scu_ipc_register_write(u32 addr, u32 value)
{
	u32 err = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	ipc_write_dptr(addr);
	ipc_data_writel(value, 0);
	ipc_command(4 << 16 | IPC_CMD_INDIRECT_WR);
	err = busy_loop();
	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_register_write);

int intel_scu_ipc_simple_command(int cmd, int sub)
{
	u32 err = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	ipc_command(sub << 12 | cmd);
	err = busy_loop();
	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_simple_command);


int intel_scu_ipc_command(int cmd, int sub, u32 *in, int inlen,
							u32 *out, int outlen)
{
	u32 err = 0;
	int i = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	for (i = 0; i < inlen; i++)
		ipc_data_writel(*in++, 4 * i);

	ipc_command((sub << 12) | cmd | (inlen << 18));
	err = busy_loop();

	for (i = 0; i < outlen; i++)
		*out++ = ipc_data_readl(4 * i);

	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_command);

/*I2C commands */
#define IPC_I2C_WRITE 1 /* I2C Write command */
#define IPC_I2C_READ  2 /* I2C Read command */

int intel_scu_ipc_i2c_cntrl(u32 addr, u32 *data)
{
	u32 cmd = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	cmd = (addr >> 24) & 0xFF;
	if (cmd == IPC_I2C_READ) {
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
		/* Write not getting updated without delay */
		mdelay(1);
		*data = readl(ipcdev.i2c_base + I2C_DATA_ADDR);
	} else if (cmd == IPC_I2C_WRITE) {
		writel(addr, ipcdev.i2c_base + I2C_DATA_ADDR);
		mdelay(1);
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
	} else {
		dev_err(&ipcdev.pdev->dev,
			"intel_scu_ipc: I2C INVALID_CMD = 0x%x\n", cmd);

		mutex_unlock(&ipclock);
		return -1;
	}
	mutex_unlock(&ipclock);
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_i2c_cntrl);

#define IPC_FW_LOAD_ADDR 0xFFFC0000 /* Storage location for FW image */
#define IPC_FW_UPDATE_MBOX_ADDR 0xFFFFDFF4 /* Mailbox between ipc and scu */
#define IPC_MAX_FW_SIZE 262144 /* 256K storage size for loading the FW image */
#define IPC_FW_MIP_HEADER_SIZE 2048 /* Firmware MIP header size */
/* IPC inform SCU to get ready for update process */
#define IPC_CMD_FW_UPDATE_READY  0x10FE
/* IPC inform SCU to go for update process */
#define IPC_CMD_FW_UPDATE_GO     0x20FE
/* Status code for fw update */
#define IPC_FW_UPDATE_SUCCESS	0x444f4e45 /* Status code 'DONE' */
#define IPC_FW_UPDATE_BADN	0x4241444E /* Status code 'BADN' */
#define IPC_FW_TXHIGH		0x54784849 /* Status code 'IPC_FW_TXHIGH' */
#define IPC_FW_TXLOW		0x54784c4f /* Status code 'IPC_FW_TXLOW' */

struct fw_update_mailbox {
	u32    status;
	u32    scu_flag;
	u32    driver_flag;
};


int intel_scu_ipc_fw_update(u8 *buffer, u32 length)
{
	void __iomem *fw_update_base;
	struct fw_update_mailbox __iomem *mailbox = NULL;
	int retry_cnt = 0;
	u32 status;

	mutex_lock(&ipclock);
	fw_update_base = ioremap_nocache(IPC_FW_LOAD_ADDR, (128*1024));
	if (fw_update_base == NULL) {
		mutex_unlock(&ipclock);
		return -ENOMEM;
	}
	mailbox = ioremap_nocache(IPC_FW_UPDATE_MBOX_ADDR,
					sizeof(struct fw_update_mailbox));
	if (mailbox == NULL) {
		iounmap(fw_update_base);
		mutex_unlock(&ipclock);
		return -ENOMEM;
	}

	ipc_command(IPC_CMD_FW_UPDATE_READY);

	/* Intitialize mailbox */
	writel(0, &mailbox->status);
	writel(0, &mailbox->scu_flag);
	writel(0, &mailbox->driver_flag);

	/* Driver copies the 2KB MIP header to SRAM at 0xFFFC0000*/
	memcpy_toio(fw_update_base, buffer, 0x800);

	/* Driver sends "FW Update" IPC command (CMD_ID 0xFE; MSG_ID 0x02).
	* Upon receiving this command, SCU will write the 2K MIP header
	* from 0xFFFC0000 into NAND.
	* SCU will write a status code into the Mailbox, and then set scu_flag.
	*/

	ipc_command(IPC_CMD_FW_UPDATE_GO);

	/*Driver stalls until scu_flag is set */
	while (readl(&mailbox->scu_flag) != 1) {
		rmb();
		mdelay(1);
	}

	/* Driver checks Mailbox status.
	 * If the status is 'BADN', then abort (bad NAND).
	 * If the status is 'IPC_FW_TXLOW', then continue.
	 */
	while (readl(&mailbox->status) != IPC_FW_TXLOW) {
		rmb();
		mdelay(10);
	}
	mdelay(10);

update_retry:
	if (retry_cnt > 5)
		goto update_end;

	if (readl(&mailbox->status) != IPC_FW_TXLOW)
		goto update_end;
	buffer = buffer + 0x800;
	memcpy_toio(fw_update_base, buffer, 0x20000);
	writel(1, &mailbox->driver_flag);
	while (readl(&mailbox->scu_flag) == 1) {
		rmb();
		mdelay(1);
	}

	/* check for 'BADN' */
	if (readl(&mailbox->status) == IPC_FW_UPDATE_BADN)
		goto update_end;

	while (readl(&mailbox->status) != IPC_FW_TXHIGH) {
		rmb();
		mdelay(10);
	}
	mdelay(10);

	if (readl(&mailbox->status) != IPC_FW_TXHIGH)
		goto update_end;

	buffer = buffer + 0x20000;
	memcpy_toio(fw_update_base, buffer, 0x20000);
	writel(0, &mailbox->driver_flag);

	while (mailbox->scu_flag == 0) {
		rmb();
		mdelay(1);
	}

	/* check for 'BADN' */
	if (readl(&mailbox->status) == IPC_FW_UPDATE_BADN)
		goto update_end;

	if (readl(&mailbox->status) == IPC_FW_TXLOW) {
		++retry_cnt;
		goto update_retry;
	}

update_end:
	status = readl(&mailbox->status);

	iounmap(fw_update_base);
	iounmap(mailbox);
	mutex_unlock(&ipclock);

	if (status == IPC_FW_UPDATE_SUCCESS)
		return 0;
	return -1;
}
EXPORT_SYMBOL(intel_scu_ipc_fw_update);

static irqreturn_t ioc(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int err;
	resource_size_t pci_resource;

	if (ipcdev.pdev)		/* We support only one SCU */
		return -EBUSY;

	ipcdev.pdev = pci_dev_get(dev);

	err = pci_enable_device(dev);
	if (err)
		return err;

	err = pci_request_regions(dev, "intel_scu_ipc");
	if (err)
		return err;

	pci_resource = pci_resource_start(dev, 0);
	if (!pci_resource)
		return -ENOMEM;

	if (request_irq(dev->irq, ioc, 0, "intel_scu_ipc", &ipcdev))
		return -EBUSY;

	ipcdev.ipc_base = ioremap_nocache(IPC_BASE_ADDR, IPC_MAX_ADDR);
	if (!ipcdev.ipc_base)
		return -ENOMEM;

	ipcdev.i2c_base = ioremap_nocache(IPC_I2C_BASE, IPC_I2C_MAX_ADDR);
	if (!ipcdev.i2c_base) {
		iounmap(ipcdev.ipc_base);
		return -ENOMEM;
	}
	return 0;
}

static void ipc_remove(struct pci_dev *pdev)
{
	free_irq(pdev->irq, &ipcdev);
	pci_release_regions(pdev);
	pci_dev_put(ipcdev.pdev);
	iounmap(ipcdev.ipc_base);
	iounmap(ipcdev.i2c_base);
	ipcdev.pdev = NULL;
}

static const struct pci_device_id pci_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x080e)},
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver ipc_driver = {
	.name = "intel_scu_ipc",
	.id_table = pci_ids,
	.probe = ipc_probe,
	.remove = ipc_remove,
};


static int __init intel_scu_ipc_init(void)
{
	return  pci_register_driver(&ipc_driver);
}

static void __exit intel_scu_ipc_exit(void)
{
	pci_unregister_driver(&ipc_driver);
}

MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_DESCRIPTION("Intel SCU IPC driver");
MODULE_LICENSE("GPL");

module_init(intel_scu_ipc_init);
module_exit(intel_scu_ipc_exit);
