


#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>

#include "a100u2w.h"


static struct orc_scb *__orc_alloc_scb(struct orc_host * host);
static void inia100_scb_handler(struct orc_host *host, struct orc_scb *scb);

static struct orc_nvram nvram, *nvramp = &nvram;

static u8 default_nvram[64] =
{
/*----------header -------------*/
	0x01,			/* 0x00: Sub System Vendor ID 0 */
	0x11,			/* 0x01: Sub System Vendor ID 1 */
	0x60,			/* 0x02: Sub System ID 0        */
	0x10,			/* 0x03: Sub System ID 1        */
	0x00,			/* 0x04: SubClass               */
	0x01,			/* 0x05: Vendor ID 0            */
	0x11,			/* 0x06: Vendor ID 1            */
	0x60,			/* 0x07: Device ID 0            */
	0x10,			/* 0x08: Device ID 1            */
	0x00,			/* 0x09: Reserved               */
	0x00,			/* 0x0A: Reserved               */
	0x01,			/* 0x0B: Revision of Data Structure     */
				/* -- Host Adapter Structure --- */
	0x01,			/* 0x0C: Number Of SCSI Channel */
	0x01,			/* 0x0D: BIOS Configuration 1   */
	0x00,			/* 0x0E: BIOS Configuration 2   */
	0x00,			/* 0x0F: BIOS Configuration 3   */
				/* --- SCSI Channel 0 Configuration --- */
	0x07,			/* 0x10: H/A ID                 */
	0x83,			/* 0x11: Channel Configuration  */
	0x20,			/* 0x12: MAX TAG per target     */
	0x0A,			/* 0x13: SCSI Reset Recovering time     */
	0x00,			/* 0x14: Channel Configuration4 */
	0x00,			/* 0x15: Channel Configuration5 */
				/* SCSI Channel 0 Target Configuration  */
				/* 0x16-0x25                    */
	0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8,
	0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8,
				/* --- SCSI Channel 1 Configuration --- */
	0x07,			/* 0x26: H/A ID                 */
	0x83,			/* 0x27: Channel Configuration  */
	0x20,			/* 0x28: MAX TAG per target     */
	0x0A,			/* 0x29: SCSI Reset Recovering time     */
	0x00,			/* 0x2A: Channel Configuration4 */
	0x00,			/* 0x2B: Channel Configuration5 */
				/* SCSI Channel 1 Target Configuration  */
				/* 0x2C-0x3B                    */
	0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8,
	0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8,
	0x00,			/* 0x3C: Reserved               */
	0x00,			/* 0x3D: Reserved               */
	0x00,			/* 0x3E: Reserved               */
	0x00			/* 0x3F: Checksum               */
};


static u8 wait_chip_ready(struct orc_host * host)
{
	int i;

	for (i = 0; i < 10; i++) {	/* Wait 1 second for report timeout     */
		if (inb(host->base + ORC_HCTRL) & HOSTSTOP)	/* Wait HOSTSTOP set */
			return 1;
		mdelay(100);
	}
	return 0;
}

static u8 wait_firmware_ready(struct orc_host * host)
{
	int i;

	for (i = 0; i < 10; i++) {	/* Wait 1 second for report timeout     */
		if (inb(host->base + ORC_HSTUS) & RREADY)		/* Wait READY set */
			return 1;
		mdelay(100);	/* wait 100ms before try again  */
	}
	return 0;
}

/***************************************************************************/
static u8 wait_scsi_reset_done(struct orc_host * host)
{
	int i;

	for (i = 0; i < 10; i++) {	/* Wait 1 second for report timeout     */
		if (!(inb(host->base + ORC_HCTRL) & SCSIRST))	/* Wait SCSIRST done */
			return 1;
		mdelay(100);	/* wait 100ms before try again  */
	}
	return 0;
}

/***************************************************************************/
static u8 wait_HDO_off(struct orc_host * host)
{
	int i;

	for (i = 0; i < 10; i++) {	/* Wait 1 second for report timeout     */
		if (!(inb(host->base + ORC_HCTRL) & HDO))		/* Wait HDO off */
			return 1;
		mdelay(100);	/* wait 100ms before try again  */
	}
	return 0;
}

/***************************************************************************/
static u8 wait_hdi_set(struct orc_host * host, u8 * data)
{
	int i;

	for (i = 0; i < 10; i++) {	/* Wait 1 second for report timeout     */
		if ((*data = inb(host->base + ORC_HSTUS)) & HDI)
			return 1;	/* Wait HDI set */
		mdelay(100);	/* wait 100ms before try again  */
	}
	return 0;
}

/***************************************************************************/
static unsigned short orc_read_fwrev(struct orc_host * host)
{
	u16 version;
	u8 data;

	outb(ORC_CMD_VERSION, host->base + ORC_HDATA);
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	if (wait_hdi_set(host, &data) == 0)	/* Wait HDI set   */
		return 0;
	version = inb(host->base + ORC_HDATA);
	outb(data, host->base + ORC_HSTUS);	/* Clear HDI            */

	if (wait_hdi_set(host, &data) == 0)	/* Wait HDI set   */
		return 0;
	version |= inb(host->base + ORC_HDATA) << 8;
	outb(data, host->base + ORC_HSTUS);	/* Clear HDI            */

	return version;
}

/***************************************************************************/
static u8 orc_nv_write(struct orc_host * host, unsigned char address, unsigned char value)
{
	outb(ORC_CMD_SET_NVM, host->base + ORC_HDATA);	/* Write command */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	outb(address, host->base + ORC_HDATA);	/* Write address */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	outb(value, host->base + ORC_HDATA);	/* Write value  */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	return 1;
}

/***************************************************************************/
static u8 orc_nv_read(struct orc_host * host, u8 address, u8 *ptr)
{
	unsigned char data;

	outb(ORC_CMD_GET_NVM, host->base + ORC_HDATA);	/* Write command */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	outb(address, host->base + ORC_HDATA);	/* Write address */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	if (wait_hdi_set(host, &data) == 0)	/* Wait HDI set   */
		return 0;
	*ptr = inb(host->base + ORC_HDATA);
	outb(data, host->base + ORC_HSTUS);	/* Clear HDI    */

	return 1;

}


static void orc_exec_scb(struct orc_host * host, struct orc_scb * scb)
{
	scb->status = ORCSCB_POST;
	outb(scb->scbidx, host->base + ORC_PQUEUE);
}



static int se2_rd_all(struct orc_host * host)
{
	int i;
	u8 *np, chksum = 0;

	np = (u8 *) nvramp;
	for (i = 0; i < 64; i++, np++) {	/* <01> */
		if (orc_nv_read(host, (u8) i, np) == 0)
			return -1;
	}

	/*------ Is ckecksum ok ? ------*/
	np = (u8 *) nvramp;
	for (i = 0; i < 63; i++)
		chksum += *np++;

	if (nvramp->CheckSum != (u8) chksum)
		return -1;
	return 1;
}


static void se2_update_all(struct orc_host * host)
{				/* setup default pattern  */
	int i;
	u8 *np, *np1, chksum = 0;

	/* Calculate checksum first   */
	np = (u8 *) default_nvram;
	for (i = 0; i < 63; i++)
		chksum += *np++;
	*np = chksum;

	np = (u8 *) default_nvram;
	np1 = (u8 *) nvramp;
	for (i = 0; i < 64; i++, np++, np1++) {
		if (*np != *np1)
			orc_nv_write(host, (u8) i, *np);
	}
}


static void read_eeprom(struct orc_host * host)
{
	if (se2_rd_all(host) != 1) {
		se2_update_all(host);	/* setup default pattern        */
		se2_rd_all(host);	/* load again                   */
	}
}



static u8 orc_load_firmware(struct orc_host * host)
{
	u32 data32;
	u16 bios_addr;
	u16 i;
	u8 *data32_ptr, data;


	/* Set up the EEPROM for access */

	data = inb(host->base + ORC_GCFG);
	outb(data | EEPRG, host->base + ORC_GCFG);	/* Enable EEPROM programming */
	outb(0x00, host->base + ORC_EBIOSADR2);
	outw(0x0000, host->base + ORC_EBIOSADR0);
	if (inb(host->base + ORC_EBIOSDATA) != 0x55) {
		outb(data, host->base + ORC_GCFG);	/* Disable EEPROM programming */
		return 0;
	}
	outw(0x0001, host->base + ORC_EBIOSADR0);
	if (inb(host->base + ORC_EBIOSDATA) != 0xAA) {
		outb(data, host->base + ORC_GCFG);	/* Disable EEPROM programming */
		return 0;
	}

	outb(PRGMRST | DOWNLOAD, host->base + ORC_RISCCTL);	/* Enable SRAM programming */
	data32_ptr = (u8 *) & data32;
	data32 = cpu_to_le32(0);		/* Initial FW address to 0 */
	outw(0x0010, host->base + ORC_EBIOSADR0);
	*data32_ptr = inb(host->base + ORC_EBIOSDATA);		/* Read from BIOS */
	outw(0x0011, host->base + ORC_EBIOSADR0);
	*(data32_ptr + 1) = inb(host->base + ORC_EBIOSDATA);	/* Read from BIOS */
	outw(0x0012, host->base + ORC_EBIOSADR0);
	*(data32_ptr + 2) = inb(host->base + ORC_EBIOSDATA);	/* Read from BIOS */
	outw(*(data32_ptr + 2), host->base + ORC_EBIOSADR2);
	outl(le32_to_cpu(data32), host->base + ORC_FWBASEADR);		/* Write FW address */

	/* Copy the code from the BIOS to the SRAM */

	udelay(500);	/* Required on Sun Ultra 5 ... 350 -> failures */
	bios_addr = (u16) le32_to_cpu(data32);	/* FW code locate at BIOS address + ? */
	for (i = 0, data32_ptr = (u8 *) & data32;	/* Download the code    */
	     i < 0x1000;	/* Firmware code size = 4K      */
	     i++, bios_addr++) {
		outw(bios_addr, host->base + ORC_EBIOSADR0);
		*data32_ptr++ = inb(host->base + ORC_EBIOSDATA);	/* Read from BIOS */
		if ((i % 4) == 3) {
			outl(le32_to_cpu(data32), host->base + ORC_RISCRAM);	/* Write every 4 bytes */
			data32_ptr = (u8 *) & data32;
		}
	}

	/* Go back and check they match */

	outb(PRGMRST | DOWNLOAD, host->base + ORC_RISCCTL);	/* Reset program count 0 */
	bios_addr -= 0x1000;	/* Reset the BIOS adddress      */
	for (i = 0, data32_ptr = (u8 *) & data32;	/* Check the code       */
	     i < 0x1000;	/* Firmware code size = 4K      */
	     i++, bios_addr++) {
		outw(bios_addr, host->base + ORC_EBIOSADR0);
		*data32_ptr++ = inb(host->base + ORC_EBIOSDATA);	/* Read from BIOS */
		if ((i % 4) == 3) {
			if (inl(host->base + ORC_RISCRAM) != le32_to_cpu(data32)) {
				outb(PRGMRST, host->base + ORC_RISCCTL);	/* Reset program to 0 */
				outb(data, host->base + ORC_GCFG);	/*Disable EEPROM programming */
				return 0;
			}
			data32_ptr = (u8 *) & data32;
		}
	}

	/* Success */
	outb(PRGMRST, host->base + ORC_RISCCTL);	/* Reset program to 0   */
	outb(data, host->base + ORC_GCFG);	/* Disable EEPROM programming */
	return 1;
}

/***************************************************************************/
static void setup_SCBs(struct orc_host * host)
{
	struct orc_scb *scb;
	int i;
	struct orc_extended_scb *escb;
	dma_addr_t escb_phys;

	/* Setup SCB base and SCB Size registers */
	outb(ORC_MAXQUEUE, host->base + ORC_SCBSIZE);	/* Total number of SCBs */
	/* SCB base address 0      */
	outl(host->scb_phys, host->base + ORC_SCBBASE0);
	/* SCB base address 1      */
	outl(host->scb_phys, host->base + ORC_SCBBASE1);

	/* setup scatter list address with one buffer */
	scb = host->scb_virt;
	escb = host->escb_virt;

	for (i = 0; i < ORC_MAXQUEUE; i++) {
		escb_phys = (host->escb_phys + (sizeof(struct orc_extended_scb) * i));
		scb->sg_addr = cpu_to_le32((u32) escb_phys);
		scb->sense_addr = cpu_to_le32((u32) escb_phys);
		scb->escb = escb;
		scb->scbidx = i;
		scb++;
		escb++;
	}
}


static void init_alloc_map(struct orc_host * host)
{
	u8 i, j;

	for (i = 0; i < MAX_CHANNELS; i++) {
		for (j = 0; j < 8; j++) {
			host->allocation_map[i][j] = 0xffffffff;
		}
	}
}


static int init_orchid(struct orc_host * host)
{
	u8 *ptr;
	u16 revision;
	u8 i;

	init_alloc_map(host);
	outb(0xFF, host->base + ORC_GIMSK);	/* Disable all interrupts */

	if (inb(host->base + ORC_HSTUS) & RREADY) {	/* Orchid is ready */
		revision = orc_read_fwrev(host);
		if (revision == 0xFFFF) {
			outb(DEVRST, host->base + ORC_HCTRL);	/* Reset Host Adapter   */
			if (wait_chip_ready(host) == 0)
				return -1;
			orc_load_firmware(host);	/* Download FW                  */
			setup_SCBs(host);	/* Setup SCB base and SCB Size registers */
			outb(0x00, host->base + ORC_HCTRL);	/* clear HOSTSTOP       */
			if (wait_firmware_ready(host) == 0)
				return -1;
			/* Wait for firmware ready     */
		} else {
			setup_SCBs(host);	/* Setup SCB base and SCB Size registers */
		}
	} else {		/* Orchid is not Ready          */
		outb(DEVRST, host->base + ORC_HCTRL);	/* Reset Host Adapter   */
		if (wait_chip_ready(host) == 0)
			return -1;
		orc_load_firmware(host);	/* Download FW                  */
		setup_SCBs(host);	/* Setup SCB base and SCB Size registers */
		outb(HDO, host->base + ORC_HCTRL);	/* Do Hardware Reset &  */

		/*     clear HOSTSTOP  */
		if (wait_firmware_ready(host) == 0)		/* Wait for firmware ready      */
			return -1;
	}

	/* Load an EEProm copy into RAM */
	/* Assumes single threaded at this point */
	read_eeprom(host);

	if (nvramp->revision != 1)
		return -1;

	host->scsi_id = nvramp->scsi_id;
	host->BIOScfg = nvramp->BIOSConfig1;
	host->max_targets = MAX_TARGETS;
	ptr = (u8 *) & (nvramp->Target00Config);
	for (i = 0; i < 16; ptr++, i++) {
		host->target_flag[i] = *ptr;
		host->max_tags[i] = ORC_MAXTAGS;
	}

	if (nvramp->SCSI0Config & NCC_BUSRESET)
		host->flags |= HCF_SCSI_RESET;
	outb(0xFB, host->base + ORC_GIMSK);	/* enable RP FIFO interrupt     */
	return 0;
}


static int orc_reset_scsi_bus(struct orc_host * host)
{				/* I need Host Control Block Information */
	unsigned long flags;

	spin_lock_irqsave(&host->allocation_lock, flags);

	init_alloc_map(host);
	/* reset scsi bus */
	outb(SCSIRST, host->base + ORC_HCTRL);
	/* FIXME: We can spend up to a second with the lock held and
	   interrupts off here */
	if (wait_scsi_reset_done(host) == 0) {
		spin_unlock_irqrestore(&host->allocation_lock, flags);
		return FAILED;
	} else {
		spin_unlock_irqrestore(&host->allocation_lock, flags);
		return SUCCESS;
	}
}


static int orc_device_reset(struct orc_host * host, struct scsi_cmnd *cmd, unsigned int target)
{				/* I need Host Control Block Information */
	struct orc_scb *scb;
	struct orc_extended_scb *escb;
	struct orc_scb *host_scb;
	u8 i;
	unsigned long flags;

	spin_lock_irqsave(&(host->allocation_lock), flags);
	scb = (struct orc_scb *) NULL;
	escb = (struct orc_extended_scb *) NULL;

	/* setup scatter list address with one buffer */
	host_scb = host->scb_virt;

	/* FIXME: is this safe if we then fail to issue the reset or race
	   a completion ? */
	init_alloc_map(host);

	/* Find the scb corresponding to the command */
	for (i = 0; i < ORC_MAXQUEUE; i++) {
		escb = host_scb->escb;
		if (host_scb->status && escb->srb == cmd)
			break;
		host_scb++;
	}

	if (i == ORC_MAXQUEUE) {
		printk(KERN_ERR "Unable to Reset - No SCB Found\n");
		spin_unlock_irqrestore(&(host->allocation_lock), flags);
		return FAILED;
	}

	/* Allocate a new SCB for the reset command to the firmware */
	if ((scb = __orc_alloc_scb(host)) == NULL) {
		/* Can't happen.. */
		spin_unlock_irqrestore(&(host->allocation_lock), flags);
		return FAILED;
	}

	/* Reset device is handled by the firmware, we fill in an SCB and
	   fire it at the controller, it does the rest */
	scb->opcode = ORC_BUSDEVRST;
	scb->target = target;
	scb->hastat = 0;
	scb->tastat = 0;
	scb->status = 0x0;
	scb->link = 0xFF;
	scb->reserved0 = 0;
	scb->reserved1 = 0;
	scb->xferlen = cpu_to_le32(0);
	scb->sg_len = cpu_to_le32(0);

	escb->srb = NULL;
	escb->srb = cmd;
	orc_exec_scb(host, scb);	/* Start execute SCB            */
	spin_unlock_irqrestore(&host->allocation_lock, flags);
	return SUCCESS;
}



static struct orc_scb *__orc_alloc_scb(struct orc_host * host)
{
	u8 channel;
	unsigned long idx;
	u8 index;
	u8 i;

	channel = host->index;
	for (i = 0; i < 8; i++) {
		for (index = 0; index < 32; index++) {
			if ((host->allocation_map[channel][i] >> index) & 0x01) {
				host->allocation_map[channel][i] &= ~(1 << index);
				idx = index + 32 * i;
				/*
				 * Translate the index to a structure instance
				 */
				return host->scb_virt + idx;
			}
		}
	}
	return NULL;
}


static struct orc_scb *orc_alloc_scb(struct orc_host * host)
{
	struct orc_scb *scb;
	unsigned long flags;

	spin_lock_irqsave(&host->allocation_lock, flags);
	scb = __orc_alloc_scb(host);
	spin_unlock_irqrestore(&host->allocation_lock, flags);
	return scb;
}


static void orc_release_scb(struct orc_host *host, struct orc_scb *scb)
{
	unsigned long flags;
	u8 index, i, channel;

	spin_lock_irqsave(&(host->allocation_lock), flags);
	channel = host->index;	/* Channel */
	index = scb->scbidx;
	i = index / 32;
	index %= 32;
	host->allocation_map[channel][i] |= (1 << index);
	spin_unlock_irqrestore(&(host->allocation_lock), flags);
}


static int orchid_abort_scb(struct orc_host * host, struct orc_scb * scb)
{
	unsigned char data, status;

	outb(ORC_CMD_ABORT_SCB, host->base + ORC_HDATA);	/* Write command */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	outb(scb->scbidx, host->base + ORC_HDATA);	/* Write address */
	outb(HDO, host->base + ORC_HCTRL);
	if (wait_HDO_off(host) == 0)	/* Wait HDO off   */
		return 0;

	if (wait_hdi_set(host, &data) == 0)	/* Wait HDI set   */
		return 0;
	status = inb(host->base + ORC_HDATA);
	outb(data, host->base + ORC_HSTUS);	/* Clear HDI    */

	if (status == 1)	/* 0 - Successfully               */
		return 0;	/* 1 - Fail                     */
	return 1;
}

static int inia100_abort_cmd(struct orc_host * host, struct scsi_cmnd *cmd)
{
	struct orc_extended_scb *escb;
	struct orc_scb *scb;
	u8 i;
	unsigned long flags;

	spin_lock_irqsave(&(host->allocation_lock), flags);

	scb = host->scb_virt;

	/* Walk the queue until we find the SCB that belongs to the command
	   block. This isn't a performance critical path so a walk in the park
	   here does no harm */

	for (i = 0; i < ORC_MAXQUEUE; i++, scb++) {
		escb = scb->escb;
		if (scb->status && escb->srb == cmd) {
			if (scb->tag_msg == 0) {
				goto out;
			} else {
				/* Issue an ABORT to the firmware */
				if (orchid_abort_scb(host, scb)) {
					escb->srb = NULL;
					spin_unlock_irqrestore(&host->allocation_lock, flags);
					return SUCCESS;
				} else
					goto out;
			}
		}
	}
out:
	spin_unlock_irqrestore(&host->allocation_lock, flags);
	return FAILED;
}


static irqreturn_t orc_interrupt(struct orc_host * host)
{
	u8 scb_index;
	struct orc_scb *scb;

	/* Check if we have an SCB queued for servicing */
	if (inb(host->base + ORC_RQUEUECNT) == 0)
		return IRQ_NONE;

	do {
		/* Get the SCB index of the SCB to service */
		scb_index = inb(host->base + ORC_RQUEUE);

		/* Translate it back to a host pointer */
		scb = (struct orc_scb *) ((unsigned long) host->scb_virt + (unsigned long) (sizeof(struct orc_scb) * scb_index));
		scb->status = 0x0;
		/* Process the SCB */
		inia100_scb_handler(host, scb);
	} while (inb(host->base + ORC_RQUEUECNT));
	return IRQ_HANDLED;
}				/* End of I1060Interrupt() */


static int inia100_build_scb(struct orc_host * host, struct orc_scb * scb, struct scsi_cmnd * cmd)
{				/* Create corresponding SCB     */
	struct scatterlist *sg;
	struct orc_sgent *sgent;		/* Pointer to SG list           */
	int i, count_sg;
	struct orc_extended_scb *escb;

	/* Links between the escb, scb and Linux scsi midlayer cmd */
	escb = scb->escb;
	escb->srb = cmd;
	sgent = NULL;

	/* Set up the SCB to do a SCSI command block */
	scb->opcode = ORC_EXECSCSI;
	scb->flags = SCF_NO_DCHK;	/* Clear done bit               */
	scb->target = cmd->device->id;
	scb->lun = cmd->device->lun;
	scb->reserved0 = 0;
	scb->reserved1 = 0;
	scb->sg_len = cpu_to_le32(0);

	scb->xferlen = cpu_to_le32((u32) scsi_bufflen(cmd));
	sgent = (struct orc_sgent *) & escb->sglist[0];

	count_sg = scsi_dma_map(cmd);
	if (count_sg < 0)
		return count_sg;
	BUG_ON(count_sg > TOTAL_SG_ENTRY);

	/* Build the scatter gather lists */
	if (count_sg) {
		scb->sg_len = cpu_to_le32((u32) (count_sg * 8));
		scsi_for_each_sg(cmd, sg, count_sg, i) {
			sgent->base = cpu_to_le32((u32) sg_dma_address(sg));
			sgent->length = cpu_to_le32((u32) sg_dma_len(sg));
			sgent++;
		}
	} else {
		scb->sg_len = cpu_to_le32(0);
		sgent->base = cpu_to_le32(0);
		sgent->length = cpu_to_le32(0);
	}
	scb->sg_addr = (u32) scb->sense_addr;	/* sense_addr is already little endian */
	scb->hastat = 0;
	scb->tastat = 0;
	scb->link = 0xFF;
	scb->sense_len = SENSE_SIZE;
	scb->cdb_len = cmd->cmd_len;
	if (scb->cdb_len >= IMAX_CDB) {
		printk("max cdb length= %x\b", cmd->cmd_len);
		scb->cdb_len = IMAX_CDB;
	}
	scb->ident = cmd->device->lun | DISC_ALLOW;
	if (cmd->device->tagged_supported) {	/* Tag Support                  */
		scb->tag_msg = SIMPLE_QUEUE_TAG;	/* Do simple tag only   */
	} else {
		scb->tag_msg = 0;	/* No tag support               */
	}
	memcpy(scb->cdb, cmd->cmnd, scb->cdb_len);
	return 0;
}


static int inia100_queue(struct scsi_cmnd * cmd, void (*done) (struct scsi_cmnd *))
{
	struct orc_scb *scb;
	struct orc_host *host;		/* Point to Host adapter control block */

	host = (struct orc_host *) cmd->device->host->hostdata;
	cmd->scsi_done = done;
	/* Get free SCSI control block  */
	if ((scb = orc_alloc_scb(host)) == NULL)
		return SCSI_MLQUEUE_HOST_BUSY;

	if (inia100_build_scb(host, scb, cmd)) {
		orc_release_scb(host, scb);
		return SCSI_MLQUEUE_HOST_BUSY;
	}
	orc_exec_scb(host, scb);	/* Start execute SCB            */
	return 0;
}

static int inia100_abort(struct scsi_cmnd * cmd)
{
	struct orc_host *host;

	host = (struct orc_host *) cmd->device->host->hostdata;
	return inia100_abort_cmd(host, cmd);
}

static int inia100_bus_reset(struct scsi_cmnd * cmd)
{				/* I need Host Control Block Information */
	struct orc_host *host;
	host = (struct orc_host *) cmd->device->host->hostdata;
	return orc_reset_scsi_bus(host);
}

static int inia100_device_reset(struct scsi_cmnd * cmd)
{				/* I need Host Control Block Information */
	struct orc_host *host;
	host = (struct orc_host *) cmd->device->host->hostdata;
	return orc_device_reset(host, cmd, scmd_id(cmd));

}


static void inia100_scb_handler(struct orc_host *host, struct orc_scb *scb)
{
	struct scsi_cmnd *cmd;	/* Pointer to SCSI request block */
	struct orc_extended_scb *escb;

	escb = scb->escb;
	if ((cmd = (struct scsi_cmnd *) escb->srb) == NULL) {
		printk(KERN_ERR "inia100_scb_handler: SRB pointer is empty\n");
		orc_release_scb(host, scb);	/* Release SCB for current channel */
		return;
	}
	escb->srb = NULL;

	switch (scb->hastat) {
	case 0x0:
	case 0xa:		/* Linked command complete without error and linked normally */
	case 0xb:		/* Linked command complete without error interrupt generated */
		scb->hastat = 0;
		break;

	case 0x11:		/* Selection time out-The initiator selection or target
				   reselection was not complete within the SCSI Time out period */
		scb->hastat = DID_TIME_OUT;
		break;

	case 0x14:		/* Target bus phase sequence failure-An invalid bus phase or bus
				   phase sequence was requested by the target. The host adapter
				   will generate a SCSI Reset Condition, notifying the host with
				   a SCRD interrupt */
		scb->hastat = DID_RESET;
		break;

	case 0x1a:		/* SCB Aborted. 07/21/98 */
		scb->hastat = DID_ABORT;
		break;

	case 0x12:		/* Data overrun/underrun-The target attempted to transfer more data
				   than was allocated by the Data Length field or the sum of the
				   Scatter / Gather Data Length fields. */
	case 0x13:		/* Unexpected bus free-The target dropped the SCSI BSY at an unexpected time. */
	case 0x16:		/* Invalid CCB Operation Code-The first byte of the CCB was invalid. */

	default:
		printk(KERN_DEBUG "inia100: %x %x\n", scb->hastat, scb->tastat);
		scb->hastat = DID_ERROR;	/* Couldn't find any better */
		break;
	}

	if (scb->tastat == 2) {	/* Check condition              */
		memcpy((unsigned char *) &cmd->sense_buffer[0],
		   (unsigned char *) &escb->sglist[0], SENSE_SIZE);
	}
	cmd->result = scb->tastat | (scb->hastat << 16);
	scsi_dma_unmap(cmd);
	cmd->scsi_done(cmd);	/* Notify system DONE           */
	orc_release_scb(host, scb);	/* Release SCB for current channel */
}

static irqreturn_t inia100_intr(int irqno, void *devid)
{
	struct Scsi_Host *shost = (struct Scsi_Host *)devid;
	struct orc_host *host = (struct orc_host *)shost->hostdata;
	unsigned long flags;
	irqreturn_t res;

	spin_lock_irqsave(shost->host_lock, flags);
	res = orc_interrupt(host);
	spin_unlock_irqrestore(shost->host_lock, flags);

	return res;
}

static struct scsi_host_template inia100_template = {
	.proc_name		= "inia100",
	.name			= inia100_REVID,
	.queuecommand		= inia100_queue,
	.eh_abort_handler	= inia100_abort,
	.eh_bus_reset_handler	= inia100_bus_reset,
	.eh_device_reset_handler = inia100_device_reset,
	.can_queue		= 1,
	.this_id		= 1,
	.sg_tablesize		= SG_ALL,
	.cmd_per_lun 		= 1,
	.use_clustering		= ENABLE_CLUSTERING,
};

static int __devinit inia100_probe_one(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	struct Scsi_Host *shost;
	struct orc_host *host;
	unsigned long port, bios;
	int error = -ENODEV;
	u32 sz;
	unsigned long biosaddr;
	char *bios_phys;

	if (pci_enable_device(pdev))
		goto out;
	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		printk(KERN_WARNING "Unable to set 32bit DMA "
				    "on inia100 adapter, ignoring.\n");
		goto out_disable_device;
	}

	pci_set_master(pdev);

	port = pci_resource_start(pdev, 0);
	if (!request_region(port, 256, "inia100")) {
		printk(KERN_WARNING "inia100: io port 0x%lx, is busy.\n", port);
		goto out_disable_device;
	}

	/* <02> read from base address + 0x50 offset to get the bios value. */
	bios = inw(port + 0x50);


	shost = scsi_host_alloc(&inia100_template, sizeof(struct orc_host));
	if (!shost)
		goto out_release_region;

	host = (struct orc_host *)shost->hostdata;
	host->pdev = pdev;
	host->base = port;
	host->BIOScfg = bios;
	spin_lock_init(&host->allocation_lock);

	/* Get total memory needed for SCB */
	sz = ORC_MAXQUEUE * sizeof(struct orc_scb);
	host->scb_virt = pci_alloc_consistent(pdev, sz,
			&host->scb_phys);
	if (!host->scb_virt) {
		printk("inia100: SCB memory allocation error\n");
		goto out_host_put;
	}
	memset(host->scb_virt, 0, sz);

	/* Get total memory needed for ESCB */
	sz = ORC_MAXQUEUE * sizeof(struct orc_extended_scb);
	host->escb_virt = pci_alloc_consistent(pdev, sz,
			&host->escb_phys);
	if (!host->escb_virt) {
		printk("inia100: ESCB memory allocation error\n");
		goto out_free_scb_array;
	}
	memset(host->escb_virt, 0, sz);

	biosaddr = host->BIOScfg;
	biosaddr = (biosaddr << 4);
	bios_phys = phys_to_virt(biosaddr);
	if (init_orchid(host)) {	/* Initialize orchid chip */
		printk("inia100: initial orchid fail!!\n");
		goto out_free_escb_array;
	}

	shost->io_port = host->base;
	shost->n_io_port = 0xff;
	shost->can_queue = ORC_MAXQUEUE;
	shost->unique_id = shost->io_port;
	shost->max_id = host->max_targets;
	shost->max_lun = 16;
	shost->irq = pdev->irq;
	shost->this_id = host->scsi_id;	/* Assign HCS index */
	shost->sg_tablesize = TOTAL_SG_ENTRY;

	/* Initial orc chip           */
	error = request_irq(pdev->irq, inia100_intr, IRQF_SHARED,
			"inia100", shost);
	if (error < 0) {
		printk(KERN_WARNING "inia100: unable to get irq %d\n",
				pdev->irq);
		goto out_free_escb_array;
	}

	pci_set_drvdata(pdev, shost);

	error = scsi_add_host(shost, &pdev->dev);
	if (error)
		goto out_free_irq;

	scsi_scan_host(shost);
	return 0;

out_free_irq:
        free_irq(shost->irq, shost);
out_free_escb_array:
	pci_free_consistent(pdev, ORC_MAXQUEUE * sizeof(struct orc_extended_scb),
			host->escb_virt, host->escb_phys);
out_free_scb_array:
	pci_free_consistent(pdev, ORC_MAXQUEUE * sizeof(struct orc_scb),
			host->scb_virt, host->scb_phys);
out_host_put:
	scsi_host_put(shost);
out_release_region:
        release_region(port, 256);
out_disable_device:
	pci_disable_device(pdev);
out:
	return error;
}

static void __devexit inia100_remove_one(struct pci_dev *pdev)
{
	struct Scsi_Host *shost = pci_get_drvdata(pdev);
	struct orc_host *host = (struct orc_host *)shost->hostdata;

	scsi_remove_host(shost);

        free_irq(shost->irq, shost);
	pci_free_consistent(pdev, ORC_MAXQUEUE * sizeof(struct orc_extended_scb),
			host->escb_virt, host->escb_phys);
	pci_free_consistent(pdev, ORC_MAXQUEUE * sizeof(struct orc_scb),
			host->scb_virt, host->scb_phys);
        release_region(shost->io_port, 256);

	scsi_host_put(shost);
} 

static struct pci_device_id inia100_pci_tbl[] = {
	{PCI_VENDOR_ID_INIT, 0x1060, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0,}
};
MODULE_DEVICE_TABLE(pci, inia100_pci_tbl);

static struct pci_driver inia100_pci_driver = {
	.name		= "inia100",
	.id_table	= inia100_pci_tbl,
	.probe		= inia100_probe_one,
	.remove		= __devexit_p(inia100_remove_one),
};

static int __init inia100_init(void)
{
	return pci_register_driver(&inia100_pci_driver);
}

static void __exit inia100_exit(void)
{
	pci_unregister_driver(&inia100_pci_driver);
}

MODULE_DESCRIPTION("Initio A100U2W SCSI driver");
MODULE_AUTHOR("Initio Corporation");
MODULE_LICENSE("Dual BSD/GPL");

module_init(inia100_init);
module_exit(inia100_exit);
