

/* Include files */
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/eisa.h>
#include <linux/errno.h>
#include <linux/fddidevice.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tc.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "defxx.h"

/* Version information string should be updated prior to each new release!  */
#define DRV_NAME "defxx"
#define DRV_VERSION "v1.10"
#define DRV_RELDATE "2006/12/14"

static char version[] __devinitdata =
	DRV_NAME ": " DRV_VERSION " " DRV_RELDATE
	"  Lawrence V. Stefani and others\n";

#define DYNAMIC_BUFFERS 1

#define SKBUFF_RX_COPYBREAK 200
#define NEW_SKB_SIZE (PI_RCV_DATA_K_SIZE_MAX+128)

#ifdef CONFIG_PCI
#define DFX_BUS_PCI(dev) (dev->bus == &pci_bus_type)
#else
#define DFX_BUS_PCI(dev) 0
#endif

#ifdef CONFIG_EISA
#define DFX_BUS_EISA(dev) (dev->bus == &eisa_bus_type)
#else
#define DFX_BUS_EISA(dev) 0
#endif

#ifdef CONFIG_TC
#define DFX_BUS_TC(dev) (dev->bus == &tc_bus_type)
#else
#define DFX_BUS_TC(dev) 0
#endif

#ifdef CONFIG_DEFXX_MMIO
#define DFX_MMIO 1
#else
#define DFX_MMIO 0
#endif

/* Define module-wide (static) routines */

static void		dfx_bus_init(struct net_device *dev);
static void		dfx_bus_uninit(struct net_device *dev);
static void		dfx_bus_config_check(DFX_board_t *bp);

static int		dfx_driver_init(struct net_device *dev,
					const char *print_name,
					resource_size_t bar_start);
static int		dfx_adap_init(DFX_board_t *bp, int get_buffers);

static int		dfx_open(struct net_device *dev);
static int		dfx_close(struct net_device *dev);

static void		dfx_int_pr_halt_id(DFX_board_t *bp);
static void		dfx_int_type_0_process(DFX_board_t *bp);
static void		dfx_int_common(struct net_device *dev);
static irqreturn_t	dfx_interrupt(int irq, void *dev_id);

static struct		net_device_stats *dfx_ctl_get_stats(struct net_device *dev);
static void		dfx_ctl_set_multicast_list(struct net_device *dev);
static int		dfx_ctl_set_mac_address(struct net_device *dev, void *addr);
static int		dfx_ctl_update_cam(DFX_board_t *bp);
static int		dfx_ctl_update_filters(DFX_board_t *bp);

static int		dfx_hw_dma_cmd_req(DFX_board_t *bp);
static int		dfx_hw_port_ctrl_req(DFX_board_t *bp, PI_UINT32	command, PI_UINT32 data_a, PI_UINT32 data_b, PI_UINT32 *host_data);
static void		dfx_hw_adap_reset(DFX_board_t *bp, PI_UINT32 type);
static int		dfx_hw_adap_state_rd(DFX_board_t *bp);
static int		dfx_hw_dma_uninit(DFX_board_t *bp, PI_UINT32 type);

static int		dfx_rcv_init(DFX_board_t *bp, int get_buffers);
static void		dfx_rcv_queue_process(DFX_board_t *bp);
static void		dfx_rcv_flush(DFX_board_t *bp);

static netdev_tx_t dfx_xmt_queue_pkt(struct sk_buff *skb,
				     struct net_device *dev);
static int		dfx_xmt_done(DFX_board_t *bp);
static void		dfx_xmt_flush(DFX_board_t *bp);

/* Define module-wide (static) variables */

static struct pci_driver dfx_pci_driver;
static struct eisa_driver dfx_eisa_driver;
static struct tc_driver dfx_tc_driver;



static inline void dfx_writel(DFX_board_t *bp, int offset, u32 data)
{
	writel(data, bp->base.mem + offset);
	mb();
}

static inline void dfx_outl(DFX_board_t *bp, int offset, u32 data)
{
	outl(data, bp->base.port + offset);
}

static void dfx_port_write_long(DFX_board_t *bp, int offset, u32 data)
{
	struct device __maybe_unused *bdev = bp->bus_dev;
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;

	if (dfx_use_mmio)
		dfx_writel(bp, offset, data);
	else
		dfx_outl(bp, offset, data);
}


static inline void dfx_readl(DFX_board_t *bp, int offset, u32 *data)
{
	mb();
	*data = readl(bp->base.mem + offset);
}

static inline void dfx_inl(DFX_board_t *bp, int offset, u32 *data)
{
	*data = inl(bp->base.port + offset);
}

static void dfx_port_read_long(DFX_board_t *bp, int offset, u32 *data)
{
	struct device __maybe_unused *bdev = bp->bus_dev;
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;

	if (dfx_use_mmio)
		dfx_readl(bp, offset, data);
	else
		dfx_inl(bp, offset, data);
}


static void dfx_get_bars(struct device *bdev,
			 resource_size_t *bar_start, resource_size_t *bar_len)
{
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;

	if (dfx_bus_pci) {
		int num = dfx_use_mmio ? 0 : 1;

		*bar_start = pci_resource_start(to_pci_dev(bdev), num);
		*bar_len = pci_resource_len(to_pci_dev(bdev), num);
	}
	if (dfx_bus_eisa) {
		unsigned long base_addr = to_eisa_device(bdev)->base_addr;
		resource_size_t bar;

		if (dfx_use_mmio) {
			bar = inb(base_addr + PI_ESIC_K_MEM_ADD_CMP_2);
			bar <<= 8;
			bar |= inb(base_addr + PI_ESIC_K_MEM_ADD_CMP_1);
			bar <<= 8;
			bar |= inb(base_addr + PI_ESIC_K_MEM_ADD_CMP_0);
			bar <<= 16;
			*bar_start = bar;
			bar = inb(base_addr + PI_ESIC_K_MEM_ADD_MASK_2);
			bar <<= 8;
			bar |= inb(base_addr + PI_ESIC_K_MEM_ADD_MASK_1);
			bar <<= 8;
			bar |= inb(base_addr + PI_ESIC_K_MEM_ADD_MASK_0);
			bar <<= 16;
			*bar_len = (bar | PI_MEM_ADD_MASK_M) + 1;
		} else {
			*bar_start = base_addr;
			*bar_len = PI_ESIC_K_CSR_IO_LEN;
		}
	}
	if (dfx_bus_tc) {
		*bar_start = to_tc_dev(bdev)->resource.start +
			     PI_TC_K_CSR_OFFSET;
		*bar_len = PI_TC_K_CSR_LEN;
	}
}

static const struct net_device_ops dfx_netdev_ops = {
	.ndo_open		= dfx_open,
	.ndo_stop		= dfx_close,
	.ndo_start_xmit		= dfx_xmt_queue_pkt,
	.ndo_get_stats		= dfx_ctl_get_stats,
	.ndo_set_multicast_list	= dfx_ctl_set_multicast_list,
	.ndo_set_mac_address	= dfx_ctl_set_mac_address,
};

static int __devinit dfx_register(struct device *bdev)
{
	static int version_disp;
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;
	const char *print_name = dev_name(bdev);
	struct net_device *dev;
	DFX_board_t	  *bp;			/* board pointer */
	resource_size_t bar_start = 0;		/* pointer to port */
	resource_size_t bar_len = 0;		/* resource length */
	int alloc_size;				/* total buffer size used */
	struct resource *region;
	int err = 0;

	if (!version_disp) {	/* display version info if adapter is found */
		version_disp = 1;	/* set display flag to TRUE so that */
		printk(version);	/* we only display this string ONCE */
	}

	dev = alloc_fddidev(sizeof(*bp));
	if (!dev) {
		printk(KERN_ERR "%s: Unable to allocate fddidev, aborting\n",
		       print_name);
		return -ENOMEM;
	}

	/* Enable PCI device. */
	if (dfx_bus_pci && pci_enable_device(to_pci_dev(bdev))) {
		printk(KERN_ERR "%s: Cannot enable PCI device, aborting\n",
		       print_name);
		goto err_out;
	}

	SET_NETDEV_DEV(dev, bdev);

	bp = netdev_priv(dev);
	bp->bus_dev = bdev;
	dev_set_drvdata(bdev, dev);

	dfx_get_bars(bdev, &bar_start, &bar_len);

	if (dfx_use_mmio)
		region = request_mem_region(bar_start, bar_len, print_name);
	else
		region = request_region(bar_start, bar_len, print_name);
	if (!region) {
		printk(KERN_ERR "%s: Cannot reserve I/O resource "
		       "0x%lx @ 0x%lx, aborting\n",
		       print_name, (long)bar_len, (long)bar_start);
		err = -EBUSY;
		goto err_out_disable;
	}

	/* Set up I/O base address. */
	if (dfx_use_mmio) {
		bp->base.mem = ioremap_nocache(bar_start, bar_len);
		if (!bp->base.mem) {
			printk(KERN_ERR "%s: Cannot map MMIO\n", print_name);
			err = -ENOMEM;
			goto err_out_region;
		}
	} else {
		bp->base.port = bar_start;
		dev->base_addr = bar_start;
	}

	/* Initialize new device structure */
	dev->netdev_ops			= &dfx_netdev_ops;

	if (dfx_bus_pci)
		pci_set_master(to_pci_dev(bdev));

	if (dfx_driver_init(dev, print_name, bar_start) != DFX_K_SUCCESS) {
		err = -ENODEV;
		goto err_out_unmap;
	}

	err = register_netdev(dev);
	if (err)
		goto err_out_kfree;

	printk("%s: registered as %s\n", print_name, dev->name);
	return 0;

err_out_kfree:
	alloc_size = sizeof(PI_DESCR_BLOCK) +
		     PI_CMD_REQ_K_SIZE_MAX + PI_CMD_RSP_K_SIZE_MAX +
#ifndef DYNAMIC_BUFFERS
		     (bp->rcv_bufs_to_post * PI_RCV_DATA_K_SIZE_MAX) +
#endif
		     sizeof(PI_CONSUMER_BLOCK) +
		     (PI_ALIGN_K_DESC_BLK - 1);
	if (bp->kmalloced)
		dma_free_coherent(bdev, alloc_size,
				  bp->kmalloced, bp->kmalloced_dma);

err_out_unmap:
	if (dfx_use_mmio)
		iounmap(bp->base.mem);

err_out_region:
	if (dfx_use_mmio)
		release_mem_region(bar_start, bar_len);
	else
		release_region(bar_start, bar_len);

err_out_disable:
	if (dfx_bus_pci)
		pci_disable_device(to_pci_dev(bdev));

err_out:
	free_netdev(dev);
	return err;
}



static void __devinit dfx_bus_init(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);
	struct device *bdev = bp->bus_dev;
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;
	u8 val;

	DBG_printk("In dfx_bus_init...\n");

	/* Initialize a pointer back to the net_device struct */
	bp->dev = dev;

	/* Initialize adapter based on bus type */

	if (dfx_bus_tc)
		dev->irq = to_tc_dev(bdev)->interrupt;
	if (dfx_bus_eisa) {
		unsigned long base_addr = to_eisa_device(bdev)->base_addr;

		/* Get the interrupt level from the ESIC chip.  */
		val = inb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0);
		val &= PI_CONFIG_STAT_0_M_IRQ;
		val >>= PI_CONFIG_STAT_0_V_IRQ;

		switch (val) {
		case PI_CONFIG_STAT_0_IRQ_K_9:
			dev->irq = 9;
			break;

		case PI_CONFIG_STAT_0_IRQ_K_10:
			dev->irq = 10;
			break;

		case PI_CONFIG_STAT_0_IRQ_K_11:
			dev->irq = 11;
			break;

		case PI_CONFIG_STAT_0_IRQ_K_15:
			dev->irq = 15;
			break;
		}

		/*
		 * Enable memory decoding (MEMCS0) and/or port decoding
		 * (IOCS1/IOCS0) as appropriate in Function Control
		 * Register.  One of the port chip selects seems to be
		 * used for the Burst Holdoff register, but this bit of
		 * documentation is missing and as yet it has not been
		 * determined which of the two.  This is also the reason
		 * the size of the decoded port range is twice as large
		 * as one required by the PDQ.
		 */

		/* Set the decode range of the board.  */
		val = ((bp->base.port >> 12) << PI_IO_CMP_V_SLOT);
		outb(base_addr + PI_ESIC_K_IO_ADD_CMP_0_1, val);
		outb(base_addr + PI_ESIC_K_IO_ADD_CMP_0_0, 0);
		outb(base_addr + PI_ESIC_K_IO_ADD_CMP_1_1, val);
		outb(base_addr + PI_ESIC_K_IO_ADD_CMP_1_0, 0);
		val = PI_ESIC_K_CSR_IO_LEN - 1;
		outb(base_addr + PI_ESIC_K_IO_ADD_MASK_0_1, (val >> 8) & 0xff);
		outb(base_addr + PI_ESIC_K_IO_ADD_MASK_0_0, val & 0xff);
		outb(base_addr + PI_ESIC_K_IO_ADD_MASK_1_1, (val >> 8) & 0xff);
		outb(base_addr + PI_ESIC_K_IO_ADD_MASK_1_0, val & 0xff);

		/* Enable the decoders.  */
		val = PI_FUNCTION_CNTRL_M_IOCS1 | PI_FUNCTION_CNTRL_M_IOCS0;
		if (dfx_use_mmio)
			val |= PI_FUNCTION_CNTRL_M_MEMCS0;
		outb(base_addr + PI_ESIC_K_FUNCTION_CNTRL, val);

		/*
		 * Enable access to the rest of the module
		 * (including PDQ and packet memory).
		 */
		val = PI_SLOT_CNTRL_M_ENB;
		outb(base_addr + PI_ESIC_K_SLOT_CNTRL, val);

		/*
		 * Map PDQ registers into memory or port space.  This is
		 * done with a bit in the Burst Holdoff register.
		 */
		val = inb(base_addr + PI_DEFEA_K_BURST_HOLDOFF);
		if (dfx_use_mmio)
			val |= PI_BURST_HOLDOFF_V_MEM_MAP;
		else
			val &= ~PI_BURST_HOLDOFF_V_MEM_MAP;
		outb(base_addr + PI_DEFEA_K_BURST_HOLDOFF, val);

		/* Enable interrupts at EISA bus interface chip (ESIC) */
		val = inb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0);
		val |= PI_CONFIG_STAT_0_M_INT_ENB;
		outb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0, val);
	}
	if (dfx_bus_pci) {
		struct pci_dev *pdev = to_pci_dev(bdev);

		/* Get the interrupt level from the PCI Configuration Table */

		dev->irq = pdev->irq;

		/* Check Latency Timer and set if less than minimal */

		pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &val);
		if (val < PFI_K_LAT_TIMER_MIN) {
			val = PFI_K_LAT_TIMER_DEF;
			pci_write_config_byte(pdev, PCI_LATENCY_TIMER, val);
		}

		/* Enable interrupts at PCI bus interface chip (PFI) */
		val = PFI_MODE_M_PDQ_INT_ENB | PFI_MODE_M_DMA_ENB;
		dfx_port_write_long(bp, PFI_K_REG_MODE_CTRL, val);
	}
}


static void __devexit dfx_bus_uninit(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);
	struct device *bdev = bp->bus_dev;
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	u8 val;

	DBG_printk("In dfx_bus_uninit...\n");

	/* Uninitialize adapter based on bus type */

	if (dfx_bus_eisa) {
		unsigned long base_addr = to_eisa_device(bdev)->base_addr;

		/* Disable interrupts at EISA bus interface chip (ESIC) */
		val = inb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0);
		val &= ~PI_CONFIG_STAT_0_M_INT_ENB;
		outb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0, val);
	}
	if (dfx_bus_pci) {
		/* Disable interrupts at PCI bus interface chip (PFI) */
		dfx_port_write_long(bp, PFI_K_REG_MODE_CTRL, 0);
	}
}



static void __devinit dfx_bus_config_check(DFX_board_t *bp)
{
	struct device __maybe_unused *bdev = bp->bus_dev;
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	int	status;				/* return code from adapter port control call */
	u32	host_data;			/* LW data returned from port control call */

	DBG_printk("In dfx_bus_config_check...\n");

	/* Configuration check only valid for EISA adapter */

	if (dfx_bus_eisa) {
		/*
		 * First check if revision 2 EISA controller.  Rev. 1 cards used
		 * PDQ revision B, so no workaround needed in this case.  Rev. 3
		 * cards used PDQ revision E, so no workaround needed in this
		 * case, either.  Only Rev. 2 cards used either Rev. D or E
		 * chips, so we must verify the chip revision on Rev. 2 cards.
		 */
		if (to_eisa_device(bdev)->id.driver_data == DEFEA_PROD_ID_2) {
			/*
			 * Revision 2 FDDI EISA controller found,
			 * so let's check PDQ revision of adapter.
			 */
			status = dfx_hw_port_ctrl_req(bp,
											PI_PCTRL_M_SUB_CMD,
											PI_SUB_CMD_K_PDQ_REV_GET,
											0,
											&host_data);
			if ((status != DFX_K_SUCCESS) || (host_data == 2))
				{
				/*
				 * Either we couldn't determine the PDQ revision, or
				 * we determined that it is at revision D.  In either case,
				 * we need to implement the workaround.
				 */

				/* Ensure that the burst size is set to 8 longwords or less */

				switch (bp->burst_size)
					{
					case PI_PDATA_B_DMA_BURST_SIZE_32:
					case PI_PDATA_B_DMA_BURST_SIZE_16:
						bp->burst_size = PI_PDATA_B_DMA_BURST_SIZE_8;
						break;

					default:
						break;
					}

				/* Ensure that full-duplex mode is not enabled */

				bp->full_duplex_enb = PI_SNMP_K_FALSE;
				}
			}
		}
	}



static int __devinit dfx_driver_init(struct net_device *dev,
				     const char *print_name,
				     resource_size_t bar_start)
{
	DFX_board_t *bp = netdev_priv(dev);
	struct device *bdev = bp->bus_dev;
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;
	int alloc_size;			/* total buffer size needed */
	char *top_v, *curr_v;		/* virtual addrs into memory block */
	dma_addr_t top_p, curr_p;	/* physical addrs into memory block */
	u32 data;			/* host data register value */
	__le32 le32;
	char *board_name = NULL;

	DBG_printk("In dfx_driver_init...\n");

	/* Initialize bus-specific hardware registers */

	dfx_bus_init(dev);

	/*
	 * Initialize default values for configurable parameters
	 *
	 * Note: All of these parameters are ones that a user may
	 *       want to customize.  It'd be nice to break these
	 *		 out into Space.c or someplace else that's more
	 *		 accessible/understandable than this file.
	 */

	bp->full_duplex_enb		= PI_SNMP_K_FALSE;
	bp->req_ttrt			= 8 * 12500;		/* 8ms in 80 nanosec units */
	bp->burst_size			= PI_PDATA_B_DMA_BURST_SIZE_DEF;
	bp->rcv_bufs_to_post	= RCV_BUFS_DEF;

	/*
	 * Ensure that HW configuration is OK
	 *
	 * Note: Depending on the hardware revision, we may need to modify
	 *       some of the configurable parameters to workaround hardware
	 *       limitations.  We'll perform this configuration check AFTER
	 *       setting the parameters to their default values.
	 */

	dfx_bus_config_check(bp);

	/* Disable PDQ interrupts first */

	dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_DISABLE_ALL_INTS);

	/* Place adapter in DMA_UNAVAILABLE state by resetting adapter */

	(void) dfx_hw_dma_uninit(bp, PI_PDATA_A_RESET_M_SKIP_ST);

	/*  Read the factory MAC address from the adapter then save it */

	if (dfx_hw_port_ctrl_req(bp, PI_PCTRL_M_MLA, PI_PDATA_A_MLA_K_LO, 0,
				 &data) != DFX_K_SUCCESS) {
		printk("%s: Could not read adapter factory MAC address!\n",
		       print_name);
		return(DFX_K_FAILURE);
	}
	le32 = cpu_to_le32(data);
	memcpy(&bp->factory_mac_addr[0], &le32, sizeof(u32));

	if (dfx_hw_port_ctrl_req(bp, PI_PCTRL_M_MLA, PI_PDATA_A_MLA_K_HI, 0,
				 &data) != DFX_K_SUCCESS) {
		printk("%s: Could not read adapter factory MAC address!\n",
		       print_name);
		return(DFX_K_FAILURE);
	}
	le32 = cpu_to_le32(data);
	memcpy(&bp->factory_mac_addr[4], &le32, sizeof(u16));

	/*
	 * Set current address to factory address
	 *
	 * Note: Node address override support is handled through
	 *       dfx_ctl_set_mac_address.
	 */

	memcpy(dev->dev_addr, bp->factory_mac_addr, FDDI_K_ALEN);
	if (dfx_bus_tc)
		board_name = "DEFTA";
	if (dfx_bus_eisa)
		board_name = "DEFEA";
	if (dfx_bus_pci)
		board_name = "DEFPA";
	pr_info("%s: %s at %saddr = 0x%llx, IRQ = %d, Hardware addr = %pMF\n",
		print_name, board_name, dfx_use_mmio ? "" : "I/O ",
		(long long)bar_start, dev->irq, dev->dev_addr);

	/*
	 * Get memory for descriptor block, consumer block, and other buffers
	 * that need to be DMA read or written to by the adapter.
	 */

	alloc_size = sizeof(PI_DESCR_BLOCK) +
					PI_CMD_REQ_K_SIZE_MAX +
					PI_CMD_RSP_K_SIZE_MAX +
#ifndef DYNAMIC_BUFFERS
					(bp->rcv_bufs_to_post * PI_RCV_DATA_K_SIZE_MAX) +
#endif
					sizeof(PI_CONSUMER_BLOCK) +
					(PI_ALIGN_K_DESC_BLK - 1);
	bp->kmalloced = top_v = dma_alloc_coherent(bp->bus_dev, alloc_size,
						   &bp->kmalloced_dma,
						   GFP_ATOMIC);
	if (top_v == NULL) {
		printk("%s: Could not allocate memory for host buffers "
		       "and structures!\n", print_name);
		return(DFX_K_FAILURE);
	}
	memset(top_v, 0, alloc_size);	/* zero out memory before continuing */
	top_p = bp->kmalloced_dma;	/* get physical address of buffer */

	/*
	 *  To guarantee the 8K alignment required for the descriptor block, 8K - 1
	 *  plus the amount of memory needed was allocated.  The physical address
	 *	is now 8K aligned.  By carving up the memory in a specific order,
	 *  we'll guarantee the alignment requirements for all other structures.
	 *
	 *  Note: If the assumptions change regarding the non-paged, non-cached,
	 *		  physically contiguous nature of the memory block or the address
	 *		  alignments, then we'll need to implement a different algorithm
	 *		  for allocating the needed memory.
	 */

	curr_p = ALIGN(top_p, PI_ALIGN_K_DESC_BLK);
	curr_v = top_v + (curr_p - top_p);

	/* Reserve space for descriptor block */

	bp->descr_block_virt = (PI_DESCR_BLOCK *) curr_v;
	bp->descr_block_phys = curr_p;
	curr_v += sizeof(PI_DESCR_BLOCK);
	curr_p += sizeof(PI_DESCR_BLOCK);

	/* Reserve space for command request buffer */

	bp->cmd_req_virt = (PI_DMA_CMD_REQ *) curr_v;
	bp->cmd_req_phys = curr_p;
	curr_v += PI_CMD_REQ_K_SIZE_MAX;
	curr_p += PI_CMD_REQ_K_SIZE_MAX;

	/* Reserve space for command response buffer */

	bp->cmd_rsp_virt = (PI_DMA_CMD_RSP *) curr_v;
	bp->cmd_rsp_phys = curr_p;
	curr_v += PI_CMD_RSP_K_SIZE_MAX;
	curr_p += PI_CMD_RSP_K_SIZE_MAX;

	/* Reserve space for the LLC host receive queue buffers */

	bp->rcv_block_virt = curr_v;
	bp->rcv_block_phys = curr_p;

#ifndef DYNAMIC_BUFFERS
	curr_v += (bp->rcv_bufs_to_post * PI_RCV_DATA_K_SIZE_MAX);
	curr_p += (bp->rcv_bufs_to_post * PI_RCV_DATA_K_SIZE_MAX);
#endif

	/* Reserve space for the consumer block */

	bp->cons_block_virt = (PI_CONSUMER_BLOCK *) curr_v;
	bp->cons_block_phys = curr_p;

	/* Display virtual and physical addresses if debug driver */

	DBG_printk("%s: Descriptor block virt = %0lX, phys = %0X\n",
		   print_name,
		   (long)bp->descr_block_virt, bp->descr_block_phys);
	DBG_printk("%s: Command Request buffer virt = %0lX, phys = %0X\n",
		   print_name, (long)bp->cmd_req_virt, bp->cmd_req_phys);
	DBG_printk("%s: Command Response buffer virt = %0lX, phys = %0X\n",
		   print_name, (long)bp->cmd_rsp_virt, bp->cmd_rsp_phys);
	DBG_printk("%s: Receive buffer block virt = %0lX, phys = %0X\n",
		   print_name, (long)bp->rcv_block_virt, bp->rcv_block_phys);
	DBG_printk("%s: Consumer block virt = %0lX, phys = %0X\n",
		   print_name, (long)bp->cons_block_virt, bp->cons_block_phys);

	return(DFX_K_SUCCESS);
}



static int dfx_adap_init(DFX_board_t *bp, int get_buffers)
	{
	DBG_printk("In dfx_adap_init...\n");

	/* Disable PDQ interrupts first */

	dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_DISABLE_ALL_INTS);

	/* Place adapter in DMA_UNAVAILABLE state by resetting adapter */

	if (dfx_hw_dma_uninit(bp, bp->reset_type) != DFX_K_SUCCESS)
		{
		printk("%s: Could not uninitialize/reset adapter!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/*
	 * When the PDQ is reset, some false Type 0 interrupts may be pending,
	 * so we'll acknowledge all Type 0 interrupts now before continuing.
	 */

	dfx_port_write_long(bp, PI_PDQ_K_REG_TYPE_0_STATUS, PI_HOST_INT_K_ACK_ALL_TYPE_0);

	/*
	 * Clear Type 1 and Type 2 registers before going to DMA_AVAILABLE state
	 *
	 * Note: We only need to clear host copies of these registers.  The PDQ reset
	 *       takes care of the on-board register values.
	 */

	bp->cmd_req_reg.lword	= 0;
	bp->cmd_rsp_reg.lword	= 0;
	bp->rcv_xmt_reg.lword	= 0;

	/* Clear consumer block before going to DMA_AVAILABLE state */

	memset(bp->cons_block_virt, 0, sizeof(PI_CONSUMER_BLOCK));

	/* Initialize the DMA Burst Size */

	if (dfx_hw_port_ctrl_req(bp,
							PI_PCTRL_M_SUB_CMD,
							PI_SUB_CMD_K_BURST_SIZE_SET,
							bp->burst_size,
							NULL) != DFX_K_SUCCESS)
		{
		printk("%s: Could not set adapter burst size!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/*
	 * Set base address of Consumer Block
	 *
	 * Assumption: 32-bit physical address of consumer block is 64 byte
	 *			   aligned.  That is, bits 0-5 of the address must be zero.
	 */

	if (dfx_hw_port_ctrl_req(bp,
							PI_PCTRL_M_CONS_BLOCK,
							bp->cons_block_phys,
							0,
							NULL) != DFX_K_SUCCESS)
		{
		printk("%s: Could not set consumer block address!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/*
	 * Set the base address of Descriptor Block and bring adapter
	 * to DMA_AVAILABLE state.
	 *
	 * Note: We also set the literal and data swapping requirements
	 *       in this command.
	 *
	 * Assumption: 32-bit physical address of descriptor block
	 *       is 8Kbyte aligned.
	 */
	if (dfx_hw_port_ctrl_req(bp, PI_PCTRL_M_INIT,
				 (u32)(bp->descr_block_phys |
				       PI_PDATA_A_INIT_M_BSWAP_INIT),
				 0, NULL) != DFX_K_SUCCESS) {
		printk("%s: Could not set descriptor block address!\n",
		       bp->dev->name);
		return DFX_K_FAILURE;
	}

	/* Set transmit flush timeout value */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_CHARS_SET;
	bp->cmd_req_virt->char_set.item[0].item_code	= PI_ITEM_K_FLUSH_TIME;
	bp->cmd_req_virt->char_set.item[0].value		= 3;	/* 3 seconds */
	bp->cmd_req_virt->char_set.item[0].item_index	= 0;
	bp->cmd_req_virt->char_set.item[1].item_code	= PI_ITEM_K_EOL;
	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		{
		printk("%s: DMA command request failed!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/* Set the initial values for eFDXEnable and MACTReq MIB objects */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_SNMP_SET;
	bp->cmd_req_virt->snmp_set.item[0].item_code	= PI_ITEM_K_FDX_ENB_DIS;
	bp->cmd_req_virt->snmp_set.item[0].value		= bp->full_duplex_enb;
	bp->cmd_req_virt->snmp_set.item[0].item_index	= 0;
	bp->cmd_req_virt->snmp_set.item[1].item_code	= PI_ITEM_K_MAC_T_REQ;
	bp->cmd_req_virt->snmp_set.item[1].value		= bp->req_ttrt;
	bp->cmd_req_virt->snmp_set.item[1].item_index	= 0;
	bp->cmd_req_virt->snmp_set.item[2].item_code	= PI_ITEM_K_EOL;
	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		{
		printk("%s: DMA command request failed!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/* Initialize adapter CAM */

	if (dfx_ctl_update_cam(bp) != DFX_K_SUCCESS)
		{
		printk("%s: Adapter CAM update failed!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/* Initialize adapter filters */

	if (dfx_ctl_update_filters(bp) != DFX_K_SUCCESS)
		{
		printk("%s: Adapter filters update failed!\n", bp->dev->name);
		return(DFX_K_FAILURE);
		}

	/*
	 * Remove any existing dynamic buffers (i.e. if the adapter is being
	 * reinitialized)
	 */

	if (get_buffers)
		dfx_rcv_flush(bp);

	/* Initialize receive descriptor block and produce buffers */

	if (dfx_rcv_init(bp, get_buffers))
	        {
		printk("%s: Receive buffer allocation failed\n", bp->dev->name);
		if (get_buffers)
			dfx_rcv_flush(bp);
		return(DFX_K_FAILURE);
		}

	/* Issue START command and bring adapter to LINK_(UN)AVAILABLE state */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_START;
	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		{
		printk("%s: Start command failed\n", bp->dev->name);
		if (get_buffers)
			dfx_rcv_flush(bp);
		return(DFX_K_FAILURE);
		}

	/* Initialization succeeded, reenable PDQ interrupts */

	dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_ENABLE_DEF_INTS);
	return(DFX_K_SUCCESS);
	}



static int dfx_open(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);
	int ret;

	DBG_printk("In dfx_open...\n");

	/* Register IRQ - support shared interrupts by passing device ptr */

	ret = request_irq(dev->irq, dfx_interrupt, IRQF_SHARED, dev->name,
			  dev);
	if (ret) {
		printk(KERN_ERR "%s: Requested IRQ %d is busy\n", dev->name, dev->irq);
		return ret;
	}

	/*
	 * Set current address to factory MAC address
	 *
	 * Note: We've already done this step in dfx_driver_init.
	 *       However, it's possible that a user has set a node
	 *		 address override, then closed and reopened the
	 *		 adapter.  Unless we reset the device address field
	 *		 now, we'll continue to use the existing modified
	 *		 address.
	 */

	memcpy(dev->dev_addr, bp->factory_mac_addr, FDDI_K_ALEN);

	/* Clear local unicast/multicast address tables and counts */

	memset(bp->uc_table, 0, sizeof(bp->uc_table));
	memset(bp->mc_table, 0, sizeof(bp->mc_table));
	bp->uc_count = 0;
	bp->mc_count = 0;

	/* Disable promiscuous filter settings */

	bp->ind_group_prom	= PI_FSTATE_K_BLOCK;
	bp->group_prom		= PI_FSTATE_K_BLOCK;

	spin_lock_init(&bp->lock);

	/* Reset and initialize adapter */

	bp->reset_type = PI_PDATA_A_RESET_M_SKIP_ST;	/* skip self-test */
	if (dfx_adap_init(bp, 1) != DFX_K_SUCCESS)
	{
		printk(KERN_ERR "%s: Adapter open failed!\n", dev->name);
		free_irq(dev->irq, dev);
		return -EAGAIN;
	}

	/* Set device structure info */
	netif_start_queue(dev);
	return(0);
}



static int dfx_close(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);

	DBG_printk("In dfx_close...\n");

	/* Disable PDQ interrupts first */

	dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_DISABLE_ALL_INTS);

	/* Place adapter in DMA_UNAVAILABLE state by resetting adapter */

	(void) dfx_hw_dma_uninit(bp, PI_PDATA_A_RESET_M_SKIP_ST);

	/*
	 * Flush any pending transmit buffers
	 *
	 * Note: It's important that we flush the transmit buffers
	 *		 BEFORE we clear our copy of the Type 2 register.
	 *		 Otherwise, we'll have no idea how many buffers
	 *		 we need to free.
	 */

	dfx_xmt_flush(bp);

	/*
	 * Clear Type 1 and Type 2 registers after adapter reset
	 *
	 * Note: Even though we're closing the adapter, it's
	 *       possible that an interrupt will occur after
	 *		 dfx_close is called.  Without some assurance to
	 *		 the contrary we want to make sure that we don't
	 *		 process receive and transmit LLC frames and update
	 *		 the Type 2 register with bad information.
	 */

	bp->cmd_req_reg.lword	= 0;
	bp->cmd_rsp_reg.lword	= 0;
	bp->rcv_xmt_reg.lword	= 0;

	/* Clear consumer block for the same reason given above */

	memset(bp->cons_block_virt, 0, sizeof(PI_CONSUMER_BLOCK));

	/* Release all dynamically allocate skb in the receive ring. */

	dfx_rcv_flush(bp);

	/* Clear device structure flags */

	netif_stop_queue(dev);

	/* Deregister (free) IRQ */

	free_irq(dev->irq, dev);

	return(0);
}



static void dfx_int_pr_halt_id(DFX_board_t	*bp)
	{
	PI_UINT32	port_status;			/* PDQ port status register value */
	PI_UINT32	halt_id;				/* PDQ port status halt ID */

	/* Read the latest port status */

	dfx_port_read_long(bp, PI_PDQ_K_REG_PORT_STATUS, &port_status);

	/* Display halt state transition information */

	halt_id = (port_status & PI_PSTATUS_M_HALT_ID) >> PI_PSTATUS_V_HALT_ID;
	switch (halt_id)
		{
		case PI_HALT_ID_K_SELFTEST_TIMEOUT:
			printk("%s: Halt ID: Selftest Timeout\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_PARITY_ERROR:
			printk("%s: Halt ID: Host Bus Parity Error\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_HOST_DIR_HALT:
			printk("%s: Halt ID: Host-Directed Halt\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_SW_FAULT:
			printk("%s: Halt ID: Adapter Software Fault\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_HW_FAULT:
			printk("%s: Halt ID: Adapter Hardware Fault\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_PC_TRACE:
			printk("%s: Halt ID: FDDI Network PC Trace Path Test\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_DMA_ERROR:
			printk("%s: Halt ID: Adapter DMA Error\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_IMAGE_CRC_ERROR:
			printk("%s: Halt ID: Firmware Image CRC Error\n", bp->dev->name);
			break;

		case PI_HALT_ID_K_BUS_EXCEPTION:
			printk("%s: Halt ID: 68000 Bus Exception\n", bp->dev->name);
			break;

		default:
			printk("%s: Halt ID: Unknown (code = %X)\n", bp->dev->name, halt_id);
			break;
		}
	}



static void dfx_int_type_0_process(DFX_board_t	*bp)

	{
	PI_UINT32	type_0_status;		/* Host Interrupt Type 0 register */
	PI_UINT32	state;				/* current adap state (from port status) */

	/*
	 * Read host interrupt Type 0 register to determine which Type 0
	 * interrupts are pending.  Immediately write it back out to clear
	 * those interrupts.
	 */

	dfx_port_read_long(bp, PI_PDQ_K_REG_TYPE_0_STATUS, &type_0_status);
	dfx_port_write_long(bp, PI_PDQ_K_REG_TYPE_0_STATUS, type_0_status);

	/* Check for Type 0 error interrupts */

	if (type_0_status & (PI_TYPE_0_STAT_M_NXM |
							PI_TYPE_0_STAT_M_PM_PAR_ERR |
							PI_TYPE_0_STAT_M_BUS_PAR_ERR))
		{
		/* Check for Non-Existent Memory error */

		if (type_0_status & PI_TYPE_0_STAT_M_NXM)
			printk("%s: Non-Existent Memory Access Error\n", bp->dev->name);

		/* Check for Packet Memory Parity error */

		if (type_0_status & PI_TYPE_0_STAT_M_PM_PAR_ERR)
			printk("%s: Packet Memory Parity Error\n", bp->dev->name);

		/* Check for Host Bus Parity error */

		if (type_0_status & PI_TYPE_0_STAT_M_BUS_PAR_ERR)
			printk("%s: Host Bus Parity Error\n", bp->dev->name);

		/* Reset adapter and bring it back on-line */

		bp->link_available = PI_K_FALSE;	/* link is no longer available */
		bp->reset_type = 0;					/* rerun on-board diagnostics */
		printk("%s: Resetting adapter...\n", bp->dev->name);
		if (dfx_adap_init(bp, 0) != DFX_K_SUCCESS)
			{
			printk("%s: Adapter reset failed!  Disabling adapter interrupts.\n", bp->dev->name);
			dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_DISABLE_ALL_INTS);
			return;
			}
		printk("%s: Adapter reset successful!\n", bp->dev->name);
		return;
		}

	/* Check for transmit flush interrupt */

	if (type_0_status & PI_TYPE_0_STAT_M_XMT_FLUSH)
		{
		/* Flush any pending xmt's and acknowledge the flush interrupt */

		bp->link_available = PI_K_FALSE;		/* link is no longer available */
		dfx_xmt_flush(bp);						/* flush any outstanding packets */
		(void) dfx_hw_port_ctrl_req(bp,
									PI_PCTRL_M_XMT_DATA_FLUSH_DONE,
									0,
									0,
									NULL);
		}

	/* Check for adapter state change */

	if (type_0_status & PI_TYPE_0_STAT_M_STATE_CHANGE)
		{
		/* Get latest adapter state */

		state = dfx_hw_adap_state_rd(bp);	/* get adapter state */
		if (state == PI_STATE_K_HALTED)
			{
			/*
			 * Adapter has transitioned to HALTED state, try to reset
			 * adapter to bring it back on-line.  If reset fails,
			 * leave the adapter in the broken state.
			 */

			printk("%s: Controller has transitioned to HALTED state!\n", bp->dev->name);
			dfx_int_pr_halt_id(bp);			/* display halt id as string */

			/* Reset adapter and bring it back on-line */

			bp->link_available = PI_K_FALSE;	/* link is no longer available */
			bp->reset_type = 0;					/* rerun on-board diagnostics */
			printk("%s: Resetting adapter...\n", bp->dev->name);
			if (dfx_adap_init(bp, 0) != DFX_K_SUCCESS)
				{
				printk("%s: Adapter reset failed!  Disabling adapter interrupts.\n", bp->dev->name);
				dfx_port_write_long(bp, PI_PDQ_K_REG_HOST_INT_ENB, PI_HOST_INT_K_DISABLE_ALL_INTS);
				return;
				}
			printk("%s: Adapter reset successful!\n", bp->dev->name);
			}
		else if (state == PI_STATE_K_LINK_AVAIL)
			{
			bp->link_available = PI_K_TRUE;		/* set link available flag */
			}
		}
	}



static void dfx_int_common(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);
	PI_UINT32	port_status;		/* Port Status register */

	/* Process xmt interrupts - frequent case, so always call this routine */

	if(dfx_xmt_done(bp))				/* free consumed xmt packets */
		netif_wake_queue(dev);

	/* Process rcv interrupts - frequent case, so always call this routine */

	dfx_rcv_queue_process(bp);		/* service received LLC frames */

	/*
	 * Transmit and receive producer and completion indices are updated on the
	 * adapter by writing to the Type 2 Producer register.  Since the frequent
	 * case is that we'll be processing either LLC transmit or receive buffers,
	 * we'll optimize I/O writes by doing a single register write here.
	 */

	dfx_port_write_long(bp, PI_PDQ_K_REG_TYPE_2_PROD, bp->rcv_xmt_reg.lword);

	/* Read PDQ Port Status register to find out which interrupts need processing */

	dfx_port_read_long(bp, PI_PDQ_K_REG_PORT_STATUS, &port_status);

	/* Process Type 0 interrupts (if any) - infrequent, so only call when needed */

	if (port_status & PI_PSTATUS_M_TYPE_0_PENDING)
		dfx_int_type_0_process(bp);	/* process Type 0 interrupts */
	}



static irqreturn_t dfx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	DFX_board_t *bp = netdev_priv(dev);
	struct device *bdev = bp->bus_dev;
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_eisa = DFX_BUS_EISA(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);

	/* Service adapter interrupts */

	if (dfx_bus_pci) {
		u32 status;

		dfx_port_read_long(bp, PFI_K_REG_STATUS, &status);
		if (!(status & PFI_STATUS_M_PDQ_INT))
			return IRQ_NONE;

		spin_lock(&bp->lock);

		/* Disable PDQ-PFI interrupts at PFI */
		dfx_port_write_long(bp, PFI_K_REG_MODE_CTRL,
				    PFI_MODE_M_DMA_ENB);

		/* Call interrupt service routine for this adapter */
		dfx_int_common(dev);

		/* Clear PDQ interrupt status bit and reenable interrupts */
		dfx_port_write_long(bp, PFI_K_REG_STATUS,
				    PFI_STATUS_M_PDQ_INT);
		dfx_port_write_long(bp, PFI_K_REG_MODE_CTRL,
				    (PFI_MODE_M_PDQ_INT_ENB |
				     PFI_MODE_M_DMA_ENB));

		spin_unlock(&bp->lock);
	}
	if (dfx_bus_eisa) {
		unsigned long base_addr = to_eisa_device(bdev)->base_addr;
		u8 status;

		status = inb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0);
		if (!(status & PI_CONFIG_STAT_0_M_PEND))
			return IRQ_NONE;

		spin_lock(&bp->lock);

		/* Disable interrupts at the ESIC */
		status &= ~PI_CONFIG_STAT_0_M_INT_ENB;
		outb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0, status);

		/* Call interrupt service routine for this adapter */
		dfx_int_common(dev);

		/* Reenable interrupts at the ESIC */
		status = inb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0);
		status |= PI_CONFIG_STAT_0_M_INT_ENB;
		outb(base_addr + PI_ESIC_K_IO_CONFIG_STAT_0, status);

		spin_unlock(&bp->lock);
	}
	if (dfx_bus_tc) {
		u32 status;

		dfx_port_read_long(bp, PI_PDQ_K_REG_PORT_STATUS, &status);
		if (!(status & (PI_PSTATUS_M_RCV_DATA_PENDING |
				PI_PSTATUS_M_XMT_DATA_PENDING |
				PI_PSTATUS_M_SMT_HOST_PENDING |
				PI_PSTATUS_M_UNSOL_PENDING |
				PI_PSTATUS_M_CMD_RSP_PENDING |
				PI_PSTATUS_M_CMD_REQ_PENDING |
				PI_PSTATUS_M_TYPE_0_PENDING)))
			return IRQ_NONE;

		spin_lock(&bp->lock);

		/* Call interrupt service routine for this adapter */
		dfx_int_common(dev);

		spin_unlock(&bp->lock);
	}

	return IRQ_HANDLED;
}



static struct net_device_stats *dfx_ctl_get_stats(struct net_device *dev)
	{
	DFX_board_t *bp = netdev_priv(dev);

	/* Fill the bp->stats structure with driver-maintained counters */

	bp->stats.gen.rx_packets = bp->rcv_total_frames;
	bp->stats.gen.tx_packets = bp->xmt_total_frames;
	bp->stats.gen.rx_bytes   = bp->rcv_total_bytes;
	bp->stats.gen.tx_bytes   = bp->xmt_total_bytes;
	bp->stats.gen.rx_errors  = bp->rcv_crc_errors +
				   bp->rcv_frame_status_errors +
				   bp->rcv_length_errors;
	bp->stats.gen.tx_errors  = bp->xmt_length_errors;
	bp->stats.gen.rx_dropped = bp->rcv_discards;
	bp->stats.gen.tx_dropped = bp->xmt_discards;
	bp->stats.gen.multicast  = bp->rcv_multicast_frames;
	bp->stats.gen.collisions = 0;		/* always zero (0) for FDDI */

	/* Get FDDI SMT MIB objects */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_SMT_MIB_GET;
	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		return((struct net_device_stats *) &bp->stats);

	/* Fill the bp->stats structure with the SMT MIB object values */

	memcpy(bp->stats.smt_station_id, &bp->cmd_rsp_virt->smt_mib_get.smt_station_id, sizeof(bp->cmd_rsp_virt->smt_mib_get.smt_station_id));
	bp->stats.smt_op_version_id					= bp->cmd_rsp_virt->smt_mib_get.smt_op_version_id;
	bp->stats.smt_hi_version_id					= bp->cmd_rsp_virt->smt_mib_get.smt_hi_version_id;
	bp->stats.smt_lo_version_id					= bp->cmd_rsp_virt->smt_mib_get.smt_lo_version_id;
	memcpy(bp->stats.smt_user_data, &bp->cmd_rsp_virt->smt_mib_get.smt_user_data, sizeof(bp->cmd_rsp_virt->smt_mib_get.smt_user_data));
	bp->stats.smt_mib_version_id				= bp->cmd_rsp_virt->smt_mib_get.smt_mib_version_id;
	bp->stats.smt_mac_cts						= bp->cmd_rsp_virt->smt_mib_get.smt_mac_ct;
	bp->stats.smt_non_master_cts				= bp->cmd_rsp_virt->smt_mib_get.smt_non_master_ct;
	bp->stats.smt_master_cts					= bp->cmd_rsp_virt->smt_mib_get.smt_master_ct;
	bp->stats.smt_available_paths				= bp->cmd_rsp_virt->smt_mib_get.smt_available_paths;
	bp->stats.smt_config_capabilities			= bp->cmd_rsp_virt->smt_mib_get.smt_config_capabilities;
	bp->stats.smt_config_policy					= bp->cmd_rsp_virt->smt_mib_get.smt_config_policy;
	bp->stats.smt_connection_policy				= bp->cmd_rsp_virt->smt_mib_get.smt_connection_policy;
	bp->stats.smt_t_notify						= bp->cmd_rsp_virt->smt_mib_get.smt_t_notify;
	bp->stats.smt_stat_rpt_policy				= bp->cmd_rsp_virt->smt_mib_get.smt_stat_rpt_policy;
	bp->stats.smt_trace_max_expiration			= bp->cmd_rsp_virt->smt_mib_get.smt_trace_max_expiration;
	bp->stats.smt_bypass_present				= bp->cmd_rsp_virt->smt_mib_get.smt_bypass_present;
	bp->stats.smt_ecm_state						= bp->cmd_rsp_virt->smt_mib_get.smt_ecm_state;
	bp->stats.smt_cf_state						= bp->cmd_rsp_virt->smt_mib_get.smt_cf_state;
	bp->stats.smt_remote_disconnect_flag		= bp->cmd_rsp_virt->smt_mib_get.smt_remote_disconnect_flag;
	bp->stats.smt_station_status				= bp->cmd_rsp_virt->smt_mib_get.smt_station_status;
	bp->stats.smt_peer_wrap_flag				= bp->cmd_rsp_virt->smt_mib_get.smt_peer_wrap_flag;
	bp->stats.smt_time_stamp					= bp->cmd_rsp_virt->smt_mib_get.smt_msg_time_stamp.ls;
	bp->stats.smt_transition_time_stamp			= bp->cmd_rsp_virt->smt_mib_get.smt_transition_time_stamp.ls;
	bp->stats.mac_frame_status_functions		= bp->cmd_rsp_virt->smt_mib_get.mac_frame_status_functions;
	bp->stats.mac_t_max_capability				= bp->cmd_rsp_virt->smt_mib_get.mac_t_max_capability;
	bp->stats.mac_tvx_capability				= bp->cmd_rsp_virt->smt_mib_get.mac_tvx_capability;
	bp->stats.mac_available_paths				= bp->cmd_rsp_virt->smt_mib_get.mac_available_paths;
	bp->stats.mac_current_path					= bp->cmd_rsp_virt->smt_mib_get.mac_current_path;
	memcpy(bp->stats.mac_upstream_nbr, &bp->cmd_rsp_virt->smt_mib_get.mac_upstream_nbr, FDDI_K_ALEN);
	memcpy(bp->stats.mac_downstream_nbr, &bp->cmd_rsp_virt->smt_mib_get.mac_downstream_nbr, FDDI_K_ALEN);
	memcpy(bp->stats.mac_old_upstream_nbr, &bp->cmd_rsp_virt->smt_mib_get.mac_old_upstream_nbr, FDDI_K_ALEN);
	memcpy(bp->stats.mac_old_downstream_nbr, &bp->cmd_rsp_virt->smt_mib_get.mac_old_downstream_nbr, FDDI_K_ALEN);
	bp->stats.mac_dup_address_test				= bp->cmd_rsp_virt->smt_mib_get.mac_dup_address_test;
	bp->stats.mac_requested_paths				= bp->cmd_rsp_virt->smt_mib_get.mac_requested_paths;
	bp->stats.mac_downstream_port_type			= bp->cmd_rsp_virt->smt_mib_get.mac_downstream_port_type;
	memcpy(bp->stats.mac_smt_address, &bp->cmd_rsp_virt->smt_mib_get.mac_smt_address, FDDI_K_ALEN);
	bp->stats.mac_t_req							= bp->cmd_rsp_virt->smt_mib_get.mac_t_req;
	bp->stats.mac_t_neg							= bp->cmd_rsp_virt->smt_mib_get.mac_t_neg;
	bp->stats.mac_t_max							= bp->cmd_rsp_virt->smt_mib_get.mac_t_max;
	bp->stats.mac_tvx_value						= bp->cmd_rsp_virt->smt_mib_get.mac_tvx_value;
	bp->stats.mac_frame_error_threshold			= bp->cmd_rsp_virt->smt_mib_get.mac_frame_error_threshold;
	bp->stats.mac_frame_error_ratio				= bp->cmd_rsp_virt->smt_mib_get.mac_frame_error_ratio;
	bp->stats.mac_rmt_state						= bp->cmd_rsp_virt->smt_mib_get.mac_rmt_state;
	bp->stats.mac_da_flag						= bp->cmd_rsp_virt->smt_mib_get.mac_da_flag;
	bp->stats.mac_una_da_flag					= bp->cmd_rsp_virt->smt_mib_get.mac_unda_flag;
	bp->stats.mac_frame_error_flag				= bp->cmd_rsp_virt->smt_mib_get.mac_frame_error_flag;
	bp->stats.mac_ma_unitdata_available			= bp->cmd_rsp_virt->smt_mib_get.mac_ma_unitdata_available;
	bp->stats.mac_hardware_present				= bp->cmd_rsp_virt->smt_mib_get.mac_hardware_present;
	bp->stats.mac_ma_unitdata_enable			= bp->cmd_rsp_virt->smt_mib_get.mac_ma_unitdata_enable;
	bp->stats.path_tvx_lower_bound				= bp->cmd_rsp_virt->smt_mib_get.path_tvx_lower_bound;
	bp->stats.path_t_max_lower_bound			= bp->cmd_rsp_virt->smt_mib_get.path_t_max_lower_bound;
	bp->stats.path_max_t_req					= bp->cmd_rsp_virt->smt_mib_get.path_max_t_req;
	memcpy(bp->stats.path_configuration, &bp->cmd_rsp_virt->smt_mib_get.path_configuration, sizeof(bp->cmd_rsp_virt->smt_mib_get.path_configuration));
	bp->stats.port_my_type[0]					= bp->cmd_rsp_virt->smt_mib_get.port_my_type[0];
	bp->stats.port_my_type[1]					= bp->cmd_rsp_virt->smt_mib_get.port_my_type[1];
	bp->stats.port_neighbor_type[0]				= bp->cmd_rsp_virt->smt_mib_get.port_neighbor_type[0];
	bp->stats.port_neighbor_type[1]				= bp->cmd_rsp_virt->smt_mib_get.port_neighbor_type[1];
	bp->stats.port_connection_policies[0]		= bp->cmd_rsp_virt->smt_mib_get.port_connection_policies[0];
	bp->stats.port_connection_policies[1]		= bp->cmd_rsp_virt->smt_mib_get.port_connection_policies[1];
	bp->stats.port_mac_indicated[0]				= bp->cmd_rsp_virt->smt_mib_get.port_mac_indicated[0];
	bp->stats.port_mac_indicated[1]				= bp->cmd_rsp_virt->smt_mib_get.port_mac_indicated[1];
	bp->stats.port_current_path[0]				= bp->cmd_rsp_virt->smt_mib_get.port_current_path[0];
	bp->stats.port_current_path[1]				= bp->cmd_rsp_virt->smt_mib_get.port_current_path[1];
	memcpy(&bp->stats.port_requested_paths[0*3], &bp->cmd_rsp_virt->smt_mib_get.port_requested_paths[0], 3);
	memcpy(&bp->stats.port_requested_paths[1*3], &bp->cmd_rsp_virt->smt_mib_get.port_requested_paths[1], 3);
	bp->stats.port_mac_placement[0]				= bp->cmd_rsp_virt->smt_mib_get.port_mac_placement[0];
	bp->stats.port_mac_placement[1]				= bp->cmd_rsp_virt->smt_mib_get.port_mac_placement[1];
	bp->stats.port_available_paths[0]			= bp->cmd_rsp_virt->smt_mib_get.port_available_paths[0];
	bp->stats.port_available_paths[1]			= bp->cmd_rsp_virt->smt_mib_get.port_available_paths[1];
	bp->stats.port_pmd_class[0]					= bp->cmd_rsp_virt->smt_mib_get.port_pmd_class[0];
	bp->stats.port_pmd_class[1]					= bp->cmd_rsp_virt->smt_mib_get.port_pmd_class[1];
	bp->stats.port_connection_capabilities[0]	= bp->cmd_rsp_virt->smt_mib_get.port_connection_capabilities[0];
	bp->stats.port_connection_capabilities[1]	= bp->cmd_rsp_virt->smt_mib_get.port_connection_capabilities[1];
	bp->stats.port_bs_flag[0]					= bp->cmd_rsp_virt->smt_mib_get.port_bs_flag[0];
	bp->stats.port_bs_flag[1]					= bp->cmd_rsp_virt->smt_mib_get.port_bs_flag[1];
	bp->stats.port_ler_estimate[0]				= bp->cmd_rsp_virt->smt_mib_get.port_ler_estimate[0];
	bp->stats.port_ler_estimate[1]				= bp->cmd_rsp_virt->smt_mib_get.port_ler_estimate[1];
	bp->stats.port_ler_cutoff[0]				= bp->cmd_rsp_virt->smt_mib_get.port_ler_cutoff[0];
	bp->stats.port_ler_cutoff[1]				= bp->cmd_rsp_virt->smt_mib_get.port_ler_cutoff[1];
	bp->stats.port_ler_alarm[0]					= bp->cmd_rsp_virt->smt_mib_get.port_ler_alarm[0];
	bp->stats.port_ler_alarm[1]					= bp->cmd_rsp_virt->smt_mib_get.port_ler_alarm[1];
	bp->stats.port_connect_state[0]				= bp->cmd_rsp_virt->smt_mib_get.port_connect_state[0];
	bp->stats.port_connect_state[1]				= bp->cmd_rsp_virt->smt_mib_get.port_connect_state[1];
	bp->stats.port_pcm_state[0]					= bp->cmd_rsp_virt->smt_mib_get.port_pcm_state[0];
	bp->stats.port_pcm_state[1]					= bp->cmd_rsp_virt->smt_mib_get.port_pcm_state[1];
	bp->stats.port_pc_withhold[0]				= bp->cmd_rsp_virt->smt_mib_get.port_pc_withhold[0];
	bp->stats.port_pc_withhold[1]				= bp->cmd_rsp_virt->smt_mib_get.port_pc_withhold[1];
	bp->stats.port_ler_flag[0]					= bp->cmd_rsp_virt->smt_mib_get.port_ler_flag[0];
	bp->stats.port_ler_flag[1]					= bp->cmd_rsp_virt->smt_mib_get.port_ler_flag[1];
	bp->stats.port_hardware_present[0]			= bp->cmd_rsp_virt->smt_mib_get.port_hardware_present[0];
	bp->stats.port_hardware_present[1]			= bp->cmd_rsp_virt->smt_mib_get.port_hardware_present[1];

	/* Get FDDI counters */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_CNTRS_GET;
	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		return((struct net_device_stats *) &bp->stats);

	/* Fill the bp->stats structure with the FDDI counter values */

	bp->stats.mac_frame_cts				= bp->cmd_rsp_virt->cntrs_get.cntrs.frame_cnt.ls;
	bp->stats.mac_copied_cts			= bp->cmd_rsp_virt->cntrs_get.cntrs.copied_cnt.ls;
	bp->stats.mac_transmit_cts			= bp->cmd_rsp_virt->cntrs_get.cntrs.transmit_cnt.ls;
	bp->stats.mac_error_cts				= bp->cmd_rsp_virt->cntrs_get.cntrs.error_cnt.ls;
	bp->stats.mac_lost_cts				= bp->cmd_rsp_virt->cntrs_get.cntrs.lost_cnt.ls;
	bp->stats.port_lct_fail_cts[0]		= bp->cmd_rsp_virt->cntrs_get.cntrs.lct_rejects[0].ls;
	bp->stats.port_lct_fail_cts[1]		= bp->cmd_rsp_virt->cntrs_get.cntrs.lct_rejects[1].ls;
	bp->stats.port_lem_reject_cts[0]	= bp->cmd_rsp_virt->cntrs_get.cntrs.lem_rejects[0].ls;
	bp->stats.port_lem_reject_cts[1]	= bp->cmd_rsp_virt->cntrs_get.cntrs.lem_rejects[1].ls;
	bp->stats.port_lem_cts[0]			= bp->cmd_rsp_virt->cntrs_get.cntrs.link_errors[0].ls;
	bp->stats.port_lem_cts[1]			= bp->cmd_rsp_virt->cntrs_get.cntrs.link_errors[1].ls;

	return((struct net_device_stats *) &bp->stats);
	}



static void dfx_ctl_set_multicast_list(struct net_device *dev)
{
	DFX_board_t *bp = netdev_priv(dev);
	int					i;			/* used as index in for loop */
	struct netdev_hw_addr *ha;

	/* Enable LLC frame promiscuous mode, if necessary */

	if (dev->flags & IFF_PROMISC)
		bp->ind_group_prom = PI_FSTATE_K_PASS;		/* Enable LLC ind/group prom mode */

	/* Else, update multicast address table */

	else
		{
		bp->ind_group_prom = PI_FSTATE_K_BLOCK;		/* Disable LLC ind/group prom mode */
		/*
		 * Check whether incoming multicast address count exceeds table size
		 *
		 * Note: The adapters utilize an on-board 64 entry CAM for
		 *       supporting perfect filtering of multicast packets
		 *		 and bridge functions when adding unicast addresses.
		 *		 There is no hash function available.  To support
		 *		 additional multicast addresses, the all multicast
		 *		 filter (LLC group promiscuous mode) must be enabled.
		 *
		 *		 The firmware reserves two CAM entries for SMT-related
		 *		 multicast addresses, which leaves 62 entries available.
		 *		 The following code ensures that we're not being asked
		 *		 to add more than 62 addresses to the CAM.  If we are,
		 *		 the driver will enable the all multicast filter.
		 *		 Should the number of multicast addresses drop below
		 *		 the high water mark, the filter will be disabled and
		 *		 perfect filtering will be used.
		 */

		if (netdev_mc_count(dev) > (PI_CMD_ADDR_FILTER_K_SIZE - bp->uc_count))
			{
			bp->group_prom	= PI_FSTATE_K_PASS;		/* Enable LLC group prom mode */
			bp->mc_count	= 0;					/* Don't add mc addrs to CAM */
			}
		else
			{
			bp->group_prom	= PI_FSTATE_K_BLOCK;	/* Disable LLC group prom mode */
			bp->mc_count	= netdev_mc_count(dev);		/* Add mc addrs to CAM */
			}

		/* Copy addresses to multicast address table, then update adapter CAM */

		i = 0;
		netdev_for_each_mc_addr(ha, dev)
			memcpy(&bp->mc_table[i++ * FDDI_K_ALEN],
			       ha->addr, FDDI_K_ALEN);

		if (dfx_ctl_update_cam(bp) != DFX_K_SUCCESS)
			{
			DBG_printk("%s: Could not update multicast address table!\n", dev->name);
			}
		else
			{
			DBG_printk("%s: Multicast address table updated!  Added %d addresses.\n", dev->name, bp->mc_count);
			}
		}

	/* Update adapter filters */

	if (dfx_ctl_update_filters(bp) != DFX_K_SUCCESS)
		{
		DBG_printk("%s: Could not update adapter filters!\n", dev->name);
		}
	else
		{
		DBG_printk("%s: Adapter filters updated!\n", dev->name);
		}
	}



static int dfx_ctl_set_mac_address(struct net_device *dev, void *addr)
	{
	struct sockaddr	*p_sockaddr = (struct sockaddr *)addr;
	DFX_board_t *bp = netdev_priv(dev);

	/* Copy unicast address to driver-maintained structs and update count */

	memcpy(dev->dev_addr, p_sockaddr->sa_data, FDDI_K_ALEN);	/* update device struct */
	memcpy(&bp->uc_table[0], p_sockaddr->sa_data, FDDI_K_ALEN);	/* update driver struct */
	bp->uc_count = 1;

	/*
	 * Verify we're not exceeding the CAM size by adding unicast address
	 *
	 * Note: It's possible that before entering this routine we've
	 *       already filled the CAM with 62 multicast addresses.
	 *		 Since we need to place the node address override into
	 *		 the CAM, we have to check to see that we're not
	 *		 exceeding the CAM size.  If we are, we have to enable
	 *		 the LLC group (multicast) promiscuous mode filter as
	 *		 in dfx_ctl_set_multicast_list.
	 */

	if ((bp->uc_count + bp->mc_count) > PI_CMD_ADDR_FILTER_K_SIZE)
		{
		bp->group_prom	= PI_FSTATE_K_PASS;		/* Enable LLC group prom mode */
		bp->mc_count	= 0;					/* Don't add mc addrs to CAM */

		/* Update adapter filters */

		if (dfx_ctl_update_filters(bp) != DFX_K_SUCCESS)
			{
			DBG_printk("%s: Could not update adapter filters!\n", dev->name);
			}
		else
			{
			DBG_printk("%s: Adapter filters updated!\n", dev->name);
			}
		}

	/* Update adapter CAM with new unicast address */

	if (dfx_ctl_update_cam(bp) != DFX_K_SUCCESS)
		{
		DBG_printk("%s: Could not set new MAC address!\n", dev->name);
		}
	else
		{
		DBG_printk("%s: Adapter CAM updated with new MAC address\n", dev->name);
		}
	return(0);			/* always return zero */
	}



static int dfx_ctl_update_cam(DFX_board_t *bp)
	{
	int			i;				/* used as index */
	PI_LAN_ADDR	*p_addr;		/* pointer to CAM entry */

	/*
	 * Fill in command request information
	 *
	 * Note: Even though both the unicast and multicast address
	 *       table entries are stored as contiguous 6 byte entries,
	 *		 the firmware address filter set command expects each
	 *		 entry to be two longwords (8 bytes total).  We must be
	 *		 careful to only copy the six bytes of each unicast and
	 *		 multicast table entry into each command entry.  This
	 *		 is also why we must first clear the entire command
	 *		 request buffer.
	 */

	memset(bp->cmd_req_virt, 0, PI_CMD_REQ_K_SIZE_MAX);	/* first clear buffer */
	bp->cmd_req_virt->cmd_type = PI_CMD_K_ADDR_FILTER_SET;
	p_addr = &bp->cmd_req_virt->addr_filter_set.entry[0];

	/* Now add unicast addresses to command request buffer, if any */

	for (i=0; i < (int)bp->uc_count; i++)
		{
		if (i < PI_CMD_ADDR_FILTER_K_SIZE)
			{
			memcpy(p_addr, &bp->uc_table[i*FDDI_K_ALEN], FDDI_K_ALEN);
			p_addr++;			/* point to next command entry */
			}
		}

	/* Now add multicast addresses to command request buffer, if any */

	for (i=0; i < (int)bp->mc_count; i++)
		{
		if ((i + bp->uc_count) < PI_CMD_ADDR_FILTER_K_SIZE)
			{
			memcpy(p_addr, &bp->mc_table[i*FDDI_K_ALEN], FDDI_K_ALEN);
			p_addr++;			/* point to next command entry */
			}
		}

	/* Issue command to update adapter CAM, then return */

	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		return(DFX_K_FAILURE);
	return(DFX_K_SUCCESS);
	}



static int dfx_ctl_update_filters(DFX_board_t *bp)
	{
	int	i = 0;					/* used as index */

	/* Fill in command request information */

	bp->cmd_req_virt->cmd_type = PI_CMD_K_FILTERS_SET;

	/* Initialize Broadcast filter - * ALWAYS ENABLED * */

	bp->cmd_req_virt->filter_set.item[i].item_code	= PI_ITEM_K_BROADCAST;
	bp->cmd_req_virt->filter_set.item[i++].value	= PI_FSTATE_K_PASS;

	/* Initialize LLC Individual/Group Promiscuous filter */

	bp->cmd_req_virt->filter_set.item[i].item_code	= PI_ITEM_K_IND_GROUP_PROM;
	bp->cmd_req_virt->filter_set.item[i++].value	= bp->ind_group_prom;

	/* Initialize LLC Group Promiscuous filter */

	bp->cmd_req_virt->filter_set.item[i].item_code	= PI_ITEM_K_GROUP_PROM;
	bp->cmd_req_virt->filter_set.item[i++].value	= bp->group_prom;

	/* Terminate the item code list */

	bp->cmd_req_virt->filter_set.item[i].item_code	= PI_ITEM_K_EOL;

	/* Issue command to update adapter filters, then return */

	if (dfx_hw_dma_cmd_req(bp) != DFX_K_SUCCESS)
		return(DFX_K_FAILURE);
	return(DFX_K_SUCCESS);
	}



static int dfx_hw_dma_cmd_req(DFX_board_t *bp)
	{
	int status;			/* adapter status */
	int timeout_cnt;	/* used in for loops */

	/* Make sure the adapter is in a state that we can issue the DMA command in */

	status = dfx_hw_adap_state_rd(bp);
	if ((status == PI_STATE_K_RESET)		||
		(status == PI_STATE_K_HALTED)		||
		(status == PI_STATE_K_DMA_UNAVAIL)	||
		(status == PI_STATE_K_UPGRADE))
		return(DFX_K_OUTSTATE);

	/* Put response buffer on the command response queue */

	bp->descr_block_virt->cmd_rsp[bp->cmd_rsp_reg.index.prod].long_0 = (u32) (PI_RCV_DESCR_M_SOP |
			((PI_CMD_RSP_K_SIZE_MAX / PI_ALIGN_K_CMD_RSP_BUFF) << PI_RCV_DESCR_V_SEG_LEN));
	bp->descr_block_virt->cmd_rsp[bp->cmd_rsp_reg.index.prod].long_1 = bp->cmd_rsp_phys;

	/* Bump (and wrap) the producer index and write out to register */

	bp->cmd_rsp_reg.index.prod += 1;
	bp->cmd_rsp_reg.index.prod &= PI_CMD_RSP_K_NUM_ENTRIES-1;
	dfx_port_write_long(bp, PI_PDQ_K_REG_CMD_RSP_PROD, bp->cmd_rsp_reg.lword);

	/* Put request buffer on the command request queue */

	bp->descr_block_virt->cmd_req[bp->cmd_req_reg.index.prod].long_0 = (u32) (PI_XMT_DESCR_M_SOP |
			PI_XMT_DESCR_M_EOP | (PI_CMD_REQ_K_SIZE_MAX << PI_XMT_DESCR_V_SEG_LEN));
	bp->descr_block_virt->cmd_req[bp->cmd_req_reg.index.prod].long_1 = bp->cmd_req_phys;

	/* Bump (and wrap) the producer index and write out to register */

	bp->cmd_req_reg.index.prod += 1;
	bp->cmd_req_reg.index.prod &= PI_CMD_REQ_K_NUM_ENTRIES-1;
	dfx_port_write_long(bp, PI_PDQ_K_REG_CMD_REQ_PROD, bp->cmd_req_reg.lword);

	/*
	 * Here we wait for the command request consumer index to be equal
	 * to the producer, indicating that the adapter has DMAed the request.
	 */

	for (timeout_cnt = 20000; timeout_cnt > 0; timeout_cnt--)
		{
		if (bp->cmd_req_reg.index.prod == (u8)(bp->cons_block_virt->cmd_req))
			break;
		udelay(100);			/* wait for 100 microseconds */
		}
	if (timeout_cnt == 0)
		return(DFX_K_HW_TIMEOUT);

	/* Bump (and wrap) the completion index and write out to register */

	bp->cmd_req_reg.index.comp += 1;
	bp->cmd_req_reg.index.comp &= PI_CMD_REQ_K_NUM_ENTRIES-1;
	dfx_port_write_long(bp, PI_PDQ_K_REG_CMD_REQ_PROD, bp->cmd_req_reg.lword);

	/*
	 * Here we wait for the command response consumer index to be equal
	 * to the producer, indicating that the adapter has DMAed the response.
	 */

	for (timeout_cnt = 20000; timeout_cnt > 0; timeout_cnt--)
		{
		if (bp->cmd_rsp_reg.index.prod == (u8)(bp->cons_block_virt->cmd_rsp))
			break;
		udelay(100);			/* wait for 100 microseconds */
		}
	if (timeout_cnt == 0)
		return(DFX_K_HW_TIMEOUT);

	/* Bump (and wrap) the completion index and write out to register */

	bp->cmd_rsp_reg.index.comp += 1;
	bp->cmd_rsp_reg.index.comp &= PI_CMD_RSP_K_NUM_ENTRIES-1;
	dfx_port_write_long(bp, PI_PDQ_K_REG_CMD_RSP_PROD, bp->cmd_rsp_reg.lword);
	return(DFX_K_SUCCESS);
	}



static int dfx_hw_port_ctrl_req(
	DFX_board_t	*bp,
	PI_UINT32	command,
	PI_UINT32	data_a,
	PI_UINT32	data_b,
	PI_UINT32	*host_data
	)

	{
	PI_UINT32	port_cmd;		/* Port Control command register value */
	int			timeout_cnt;	/* used in for loops */

	/* Set Command Error bit in command longword */

	port_cmd = (PI_UINT32) (command | PI_PCTRL_M_CMD_ERROR);

	/* Issue port command to the adapter */

	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_DATA_A, data_a);
	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_DATA_B, data_b);
	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_CTRL, port_cmd);

	/* Now wait for command to complete */

	if (command == PI_PCTRL_M_BLAST_FLASH)
		timeout_cnt = 600000;	/* set command timeout count to 60 seconds */
	else
		timeout_cnt = 20000;	/* set command timeout count to 2 seconds */

	for (; timeout_cnt > 0; timeout_cnt--)
		{
		dfx_port_read_long(bp, PI_PDQ_K_REG_PORT_CTRL, &port_cmd);
		if (!(port_cmd & PI_PCTRL_M_CMD_ERROR))
			break;
		udelay(100);			/* wait for 100 microseconds */
		}
	if (timeout_cnt == 0)
		return(DFX_K_HW_TIMEOUT);

	/*
	 * If the address of host_data is non-zero, assume caller has supplied a
	 * non NULL pointer, and return the contents of the HOST_DATA register in
	 * it.
	 */

	if (host_data != NULL)
		dfx_port_read_long(bp, PI_PDQ_K_REG_HOST_DATA, host_data);
	return(DFX_K_SUCCESS);
	}



static void dfx_hw_adap_reset(
	DFX_board_t	*bp,
	PI_UINT32	type
	)

	{
	/* Set Reset type and assert reset */

	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_DATA_A, type);	/* tell adapter type of reset */
	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_RESET, PI_RESET_M_ASSERT_RESET);

	/* Wait for at least 1 Microsecond according to the spec. We wait 20 just to be safe */

	udelay(20);

	/* Deassert reset */

	dfx_port_write_long(bp, PI_PDQ_K_REG_PORT_RESET, 0);
	}



static int dfx_hw_adap_state_rd(DFX_board_t *bp)
	{
	PI_UINT32 port_status;		/* Port Status register value */

	dfx_port_read_long(bp, PI_PDQ_K_REG_PORT_STATUS, &port_status);
	return((port_status & PI_PSTATUS_M_STATE) >> PI_PSTATUS_V_STATE);
	}



static int dfx_hw_dma_uninit(DFX_board_t *bp, PI_UINT32 type)
	{
	int timeout_cnt;	/* used in for loops */

	/* Set reset type bit and reset adapter */

	dfx_hw_adap_reset(bp, type);

	/* Now wait for adapter to enter DMA_UNAVAILABLE state */

	for (timeout_cnt = 100000; timeout_cnt > 0; timeout_cnt--)
		{
		if (dfx_hw_adap_state_rd(bp) == PI_STATE_K_DMA_UNAVAIL)
			break;
		udelay(100);					/* wait for 100 microseconds */
		}
	if (timeout_cnt == 0)
		return(DFX_K_HW_TIMEOUT);
	return(DFX_K_SUCCESS);
	}


static void my_skb_align(struct sk_buff *skb, int n)
{
	unsigned long x = (unsigned long)skb->data;
	unsigned long v;

	v = ALIGN(x, n);	/* Where we want to be */

	skb_reserve(skb, v - x);
}



static int dfx_rcv_init(DFX_board_t *bp, int get_buffers)
	{
	int	i, j;					/* used in for loop */

	/*
	 *  Since each receive buffer is a single fragment of same length, initialize
	 *  first longword in each receive descriptor for entire LLC Host descriptor
	 *  block.  Also initialize second longword in each receive descriptor with
	 *  physical address of receive buffer.  We'll always allocate receive
	 *  buffers in powers of 2 so that we can easily fill the 256 entry descriptor
	 *  block and produce new receive buffers by simply updating the receive
	 *  producer index.
	 *
	 * 	Assumptions:
	 *		To support all shipping versions of PDQ, the receive buffer size
	 *		must be mod 128 in length and the physical address must be 128 byte
	 *		aligned.  In other words, bits 0-6 of the length and address must
	 *		be zero for the following descriptor field entries to be correct on
	 *		all PDQ-based boards.  We guaranteed both requirements during
	 *		driver initialization when we allocated memory for the receive buffers.
	 */

	if (get_buffers) {
#ifdef DYNAMIC_BUFFERS
	for (i = 0; i < (int)(bp->rcv_bufs_to_post); i++)
		for (j = 0; (i + j) < (int)PI_RCV_DATA_K_NUM_ENTRIES; j += bp->rcv_bufs_to_post)
		{
			struct sk_buff *newskb = __netdev_alloc_skb(bp->dev, NEW_SKB_SIZE, GFP_NOIO);
			if (!newskb)
				return -ENOMEM;
			bp->descr_block_virt->rcv_data[i+j].long_0 = (u32) (PI_RCV_DESCR_M_SOP |
				((PI_RCV_DATA_K_SIZE_MAX / PI_ALIGN_K_RCV_DATA_BUFF) << PI_RCV_DESCR_V_SEG_LEN));
			/*
			 * align to 128 bytes for compatibility with
			 * the old EISA boards.
			 */

			my_skb_align(newskb, 128);
			bp->descr_block_virt->rcv_data[i + j].long_1 =
				(u32)dma_map_single(bp->bus_dev, newskb->data,
						    NEW_SKB_SIZE,
						    DMA_FROM_DEVICE);
			/*
			 * p_rcv_buff_va is only used inside the
			 * kernel so we put the skb pointer here.
			 */
			bp->p_rcv_buff_va[i+j] = (char *) newskb;
		}
#else
	for (i=0; i < (int)(bp->rcv_bufs_to_post); i++)
		for (j=0; (i + j) < (int)PI_RCV_DATA_K_NUM_ENTRIES; j += bp->rcv_bufs_to_post)
			{
			bp->descr_block_virt->rcv_data[i+j].long_0 = (u32) (PI_RCV_DESCR_M_SOP |
				((PI_RCV_DATA_K_SIZE_MAX / PI_ALIGN_K_RCV_DATA_BUFF) << PI_RCV_DESCR_V_SEG_LEN));
			bp->descr_block_virt->rcv_data[i+j].long_1 = (u32) (bp->rcv_block_phys + (i * PI_RCV_DATA_K_SIZE_MAX));
			bp->p_rcv_buff_va[i+j] = (char *) (bp->rcv_block_virt + (i * PI_RCV_DATA_K_SIZE_MAX));
			}
#endif
	}

	/* Update receive producer and Type 2 register */

	bp->rcv_xmt_reg.index.rcv_prod = bp->rcv_bufs_to_post;
	dfx_port_write_long(bp, PI_PDQ_K_REG_TYPE_2_PROD, bp->rcv_xmt_reg.lword);
	return 0;
	}



static void dfx_rcv_queue_process(
	DFX_board_t *bp
	)

	{
	PI_TYPE_2_CONSUMER	*p_type_2_cons;		/* ptr to rcv/xmt consumer block register */
	char				*p_buff;			/* ptr to start of packet receive buffer (FMC descriptor) */
	u32					descr, pkt_len;		/* FMC descriptor field and packet length */
	struct sk_buff		*skb;				/* pointer to a sk_buff to hold incoming packet data */

	/* Service all consumed LLC receive frames */

	p_type_2_cons = (PI_TYPE_2_CONSUMER *)(&bp->cons_block_virt->xmt_rcv_data);
	while (bp->rcv_xmt_reg.index.rcv_comp != p_type_2_cons->index.rcv_cons)
		{
		/* Process any errors */

		int entry;

		entry = bp->rcv_xmt_reg.index.rcv_comp;
#ifdef DYNAMIC_BUFFERS
		p_buff = (char *) (((struct sk_buff *)bp->p_rcv_buff_va[entry])->data);
#else
		p_buff = (char *) bp->p_rcv_buff_va[entry];
#endif
		memcpy(&descr, p_buff + RCV_BUFF_K_DESCR, sizeof(u32));

		if (descr & PI_FMC_DESCR_M_RCC_FLUSH)
			{
			if (descr & PI_FMC_DESCR_M_RCC_CRC)
				bp->rcv_crc_errors++;
			else
				bp->rcv_frame_status_errors++;
			}
		else
		{
			int rx_in_place = 0;

			/* The frame was received without errors - verify packet length */

			pkt_len = (u32)((descr & PI_FMC_DESCR_M_LEN) >> PI_FMC_DESCR_V_LEN);
			pkt_len -= 4;				/* subtract 4 byte CRC */
			if (!IN_RANGE(pkt_len, FDDI_K_LLC_ZLEN, FDDI_K_LLC_LEN))
				bp->rcv_length_errors++;
			else{
#ifdef DYNAMIC_BUFFERS
				if (pkt_len > SKBUFF_RX_COPYBREAK) {
					struct sk_buff *newskb;

					newskb = dev_alloc_skb(NEW_SKB_SIZE);
					if (newskb){
						rx_in_place = 1;

						my_skb_align(newskb, 128);
						skb = (struct sk_buff *)bp->p_rcv_buff_va[entry];
						dma_unmap_single(bp->bus_dev,
							bp->descr_block_virt->rcv_data[entry].long_1,
							NEW_SKB_SIZE,
							DMA_FROM_DEVICE);
						skb_reserve(skb, RCV_BUFF_K_PADDING);
						bp->p_rcv_buff_va[entry] = (char *)newskb;
						bp->descr_block_virt->rcv_data[entry].long_1 =
							(u32)dma_map_single(bp->bus_dev,
								newskb->data,
								NEW_SKB_SIZE,
								DMA_FROM_DEVICE);
					} else
						skb = NULL;
				} else
#endif
					skb = dev_alloc_skb(pkt_len+3);	/* alloc new buffer to pass up, add room for PRH */
				if (skb == NULL)
					{
					printk("%s: Could not allocate receive buffer.  Dropping packet.\n", bp->dev->name);
					bp->rcv_discards++;
					break;
					}
				else {
#ifndef DYNAMIC_BUFFERS
					if (! rx_in_place)
#endif
					{
						/* Receive buffer allocated, pass receive packet up */

						skb_copy_to_linear_data(skb,
							       p_buff + RCV_BUFF_K_PADDING,
							       pkt_len + 3);
					}

					skb_reserve(skb,3);		/* adjust data field so that it points to FC byte */
					skb_put(skb, pkt_len);		/* pass up packet length, NOT including CRC */
					skb->protocol = fddi_type_trans(skb, bp->dev);
					bp->rcv_total_bytes += skb->len;
					netif_rx(skb);

					/* Update the rcv counters */
					bp->rcv_total_frames++;
					if (*(p_buff + RCV_BUFF_K_DA) & 0x01)
						bp->rcv_multicast_frames++;
				}
			}
			}

		/*
		 * Advance the producer (for recycling) and advance the completion
		 * (for servicing received frames).  Note that it is okay to
		 * advance the producer without checking that it passes the
		 * completion index because they are both advanced at the same
		 * rate.
		 */

		bp->rcv_xmt_reg.index.rcv_prod += 1;
		bp->rcv_xmt_reg.index.rcv_comp += 1;
		}
	}



static netdev_tx_t dfx_xmt_queue_pkt(struct sk_buff *skb,
				     struct net_device *dev)
	{
	DFX_board_t		*bp = netdev_priv(dev);
	u8			prod;				/* local transmit producer index */
	PI_XMT_DESCR		*p_xmt_descr;		/* ptr to transmit descriptor block entry */
	XMT_DRIVER_DESCR	*p_xmt_drv_descr;	/* ptr to transmit driver descriptor */
	unsigned long		flags;

	netif_stop_queue(dev);

	/*
	 * Verify that incoming transmit request is OK
	 *
	 * Note: The packet size check is consistent with other
	 *		 Linux device drivers, although the correct packet
	 *		 size should be verified before calling the
	 *		 transmit routine.
	 */

	if (!IN_RANGE(skb->len, FDDI_K_LLC_ZLEN, FDDI_K_LLC_LEN))
	{
		printk("%s: Invalid packet length - %u bytes\n",
			dev->name, skb->len);
		bp->xmt_length_errors++;		/* bump error counter */
		netif_wake_queue(dev);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;			/* return "success" */
	}
	/*
	 * See if adapter link is available, if not, free buffer
	 *
	 * Note: If the link isn't available, free buffer and return 0
	 *		 rather than tell the upper layer to requeue the packet.
	 *		 The methodology here is that by the time the link
	 *		 becomes available, the packet to be sent will be
	 *		 fairly stale.  By simply dropping the packet, the
	 *		 higher layer protocols will eventually time out
	 *		 waiting for response packets which it won't receive.
	 */

	if (bp->link_available == PI_K_FALSE)
		{
		if (dfx_hw_adap_state_rd(bp) == PI_STATE_K_LINK_AVAIL)	/* is link really available? */
			bp->link_available = PI_K_TRUE;		/* if so, set flag and continue */
		else
			{
			bp->xmt_discards++;					/* bump error counter */
			dev_kfree_skb(skb);		/* free sk_buff now */
			netif_wake_queue(dev);
			return NETDEV_TX_OK;		/* return "success" */
			}
		}

	spin_lock_irqsave(&bp->lock, flags);

	/* Get the current producer and the next free xmt data descriptor */

	prod		= bp->rcv_xmt_reg.index.xmt_prod;
	p_xmt_descr = &(bp->descr_block_virt->xmt_data[prod]);

	/*
	 * Get pointer to auxiliary queue entry to contain information
	 * for this packet.
	 *
	 * Note: The current xmt producer index will become the
	 *	 current xmt completion index when we complete this
	 *	 packet later on.  So, we'll get the pointer to the
	 *	 next auxiliary queue entry now before we bump the
	 *	 producer index.
	 */

	p_xmt_drv_descr = &(bp->xmt_drv_descr_blk[prod++]);	/* also bump producer index */

	/* Write the three PRH bytes immediately before the FC byte */

	skb_push(skb,3);
	skb->data[0] = DFX_PRH0_BYTE;	/* these byte values are defined */
	skb->data[1] = DFX_PRH1_BYTE;	/* in the Motorola FDDI MAC chip */
	skb->data[2] = DFX_PRH2_BYTE;	/* specification */

	/*
	 * Write the descriptor with buffer info and bump producer
	 *
	 * Note: Since we need to start DMA from the packet request
	 *		 header, we'll add 3 bytes to the DMA buffer length,
	 *		 and we'll determine the physical address of the
	 *		 buffer from the PRH, not skb->data.
	 *
	 * Assumptions:
	 *		 1. Packet starts with the frame control (FC) byte
	 *		    at skb->data.
	 *		 2. The 4-byte CRC is not appended to the buffer or
	 *			included in the length.
	 *		 3. Packet length (skb->len) is from FC to end of
	 *			data, inclusive.
	 *		 4. The packet length does not exceed the maximum
	 *			FDDI LLC frame length of 4491 bytes.
	 *		 5. The entire packet is contained in a physically
	 *			contiguous, non-cached, locked memory space
	 *			comprised of a single buffer pointed to by
	 *			skb->data.
	 *		 6. The physical address of the start of packet
	 *			can be determined from the virtual address
	 *			by using pci_map_single() and is only 32-bits
	 *			wide.
	 */

	p_xmt_descr->long_0	= (u32) (PI_XMT_DESCR_M_SOP | PI_XMT_DESCR_M_EOP | ((skb->len) << PI_XMT_DESCR_V_SEG_LEN));
	p_xmt_descr->long_1 = (u32)dma_map_single(bp->bus_dev, skb->data,
						  skb->len, DMA_TO_DEVICE);

	/*
	 * Verify that descriptor is actually available
	 *
	 * Note: If descriptor isn't available, return 1 which tells
	 *	 the upper layer to requeue the packet for later
	 *	 transmission.
	 *
	 *       We need to ensure that the producer never reaches the
	 *	 completion, except to indicate that the queue is empty.
	 */

	if (prod == bp->rcv_xmt_reg.index.xmt_comp)
	{
		skb_pull(skb,3);
		spin_unlock_irqrestore(&bp->lock, flags);
		return NETDEV_TX_BUSY;	/* requeue packet for later */
	}

	/*
	 * Save info for this packet for xmt done indication routine
	 *
	 * Normally, we'd save the producer index in the p_xmt_drv_descr
	 * structure so that we'd have it handy when we complete this
	 * packet later (in dfx_xmt_done).  However, since the current
	 * transmit architecture guarantees a single fragment for the
	 * entire packet, we can simply bump the completion index by
	 * one (1) for each completed packet.
	 *
	 * Note: If this assumption changes and we're presented with
	 *	 an inconsistent number of transmit fragments for packet
	 *	 data, we'll need to modify this code to save the current
	 *	 transmit producer index.
	 */

	p_xmt_drv_descr->p_skb = skb;

	/* Update Type 2 register */

	bp->rcv_xmt_reg.index.xmt_prod = prod;
	dfx_port_write_long(bp, PI_PDQ_K_REG_TYPE_2_PROD, bp->rcv_xmt_reg.lword);
	spin_unlock_irqrestore(&bp->lock, flags);
	netif_wake_queue(dev);
	return NETDEV_TX_OK;	/* packet queued to adapter */
	}



static int dfx_xmt_done(DFX_board_t *bp)
	{
	XMT_DRIVER_DESCR	*p_xmt_drv_descr;	/* ptr to transmit driver descriptor */
	PI_TYPE_2_CONSUMER	*p_type_2_cons;		/* ptr to rcv/xmt consumer block register */
	u8			comp;			/* local transmit completion index */
	int 			freed = 0;		/* buffers freed */

	/* Service all consumed transmit frames */

	p_type_2_cons = (PI_TYPE_2_CONSUMER *)(&bp->cons_block_virt->xmt_rcv_data);
	while (bp->rcv_xmt_reg.index.xmt_comp != p_type_2_cons->index.xmt_cons)
		{
		/* Get pointer to the transmit driver descriptor block information */

		p_xmt_drv_descr = &(bp->xmt_drv_descr_blk[bp->rcv_xmt_reg.index.xmt_comp]);

		/* Increment transmit counters */

		bp->xmt_total_frames++;
		bp->xmt_total_bytes += p_xmt_drv_descr->p_skb->len;

		/* Return skb to operating system */
		comp = bp->rcv_xmt_reg.index.xmt_comp;
		dma_unmap_single(bp->bus_dev,
				 bp->descr_block_virt->xmt_data[comp].long_1,
				 p_xmt_drv_descr->p_skb->len,
				 DMA_TO_DEVICE);
		dev_kfree_skb_irq(p_xmt_drv_descr->p_skb);

		/*
		 * Move to start of next packet by updating completion index
		 *
		 * Here we assume that a transmit packet request is always
		 * serviced by posting one fragment.  We can therefore
		 * simplify the completion code by incrementing the
		 * completion index by one.  This code will need to be
		 * modified if this assumption changes.  See comments
		 * in dfx_xmt_queue_pkt for more details.
		 */

		bp->rcv_xmt_reg.index.xmt_comp += 1;
		freed++;
		}
	return freed;
	}


#ifdef DYNAMIC_BUFFERS
static void dfx_rcv_flush( DFX_board_t *bp )
	{
	int i, j;

	for (i = 0; i < (int)(bp->rcv_bufs_to_post); i++)
		for (j = 0; (i + j) < (int)PI_RCV_DATA_K_NUM_ENTRIES; j += bp->rcv_bufs_to_post)
		{
			struct sk_buff *skb;
			skb = (struct sk_buff *)bp->p_rcv_buff_va[i+j];
			if (skb)
				dev_kfree_skb(skb);
			bp->p_rcv_buff_va[i+j] = NULL;
		}

	}
#else
static inline void dfx_rcv_flush( DFX_board_t *bp )
{
}
#endif /* DYNAMIC_BUFFERS */


static void dfx_xmt_flush( DFX_board_t *bp )
	{
	u32			prod_cons;		/* rcv/xmt consumer block longword */
	XMT_DRIVER_DESCR	*p_xmt_drv_descr;	/* ptr to transmit driver descriptor */
	u8			comp;			/* local transmit completion index */

	/* Flush all outstanding transmit frames */

	while (bp->rcv_xmt_reg.index.xmt_comp != bp->rcv_xmt_reg.index.xmt_prod)
		{
		/* Get pointer to the transmit driver descriptor block information */

		p_xmt_drv_descr = &(bp->xmt_drv_descr_blk[bp->rcv_xmt_reg.index.xmt_comp]);

		/* Return skb to operating system */
		comp = bp->rcv_xmt_reg.index.xmt_comp;
		dma_unmap_single(bp->bus_dev,
				 bp->descr_block_virt->xmt_data[comp].long_1,
				 p_xmt_drv_descr->p_skb->len,
				 DMA_TO_DEVICE);
		dev_kfree_skb(p_xmt_drv_descr->p_skb);

		/* Increment transmit error counter */

		bp->xmt_discards++;

		/*
		 * Move to start of next packet by updating completion index
		 *
		 * Here we assume that a transmit packet request is always
		 * serviced by posting one fragment.  We can therefore
		 * simplify the completion code by incrementing the
		 * completion index by one.  This code will need to be
		 * modified if this assumption changes.  See comments
		 * in dfx_xmt_queue_pkt for more details.
		 */

		bp->rcv_xmt_reg.index.xmt_comp += 1;
		}

	/* Update the transmit consumer index in the consumer block */

	prod_cons = (u32)(bp->cons_block_virt->xmt_rcv_data & ~PI_CONS_M_XMT_INDEX);
	prod_cons |= (u32)(bp->rcv_xmt_reg.index.xmt_prod << PI_CONS_V_XMT_INDEX);
	bp->cons_block_virt->xmt_rcv_data = prod_cons;
	}

static void __devexit dfx_unregister(struct device *bdev)
{
	struct net_device *dev = dev_get_drvdata(bdev);
	DFX_board_t *bp = netdev_priv(dev);
	int dfx_bus_pci = DFX_BUS_PCI(bdev);
	int dfx_bus_tc = DFX_BUS_TC(bdev);
	int dfx_use_mmio = DFX_MMIO || dfx_bus_tc;
	resource_size_t bar_start = 0;		/* pointer to port */
	resource_size_t bar_len = 0;		/* resource length */
	int		alloc_size;		/* total buffer size used */

	unregister_netdev(dev);

	alloc_size = sizeof(PI_DESCR_BLOCK) +
		     PI_CMD_REQ_K_SIZE_MAX + PI_CMD_RSP_K_SIZE_MAX +
#ifndef DYNAMIC_BUFFERS
		     (bp->rcv_bufs_to_post * PI_RCV_DATA_K_SIZE_MAX) +
#endif
		     sizeof(PI_CONSUMER_BLOCK) +
		     (PI_ALIGN_K_DESC_BLK - 1);
	if (bp->kmalloced)
		dma_free_coherent(bdev, alloc_size,
				  bp->kmalloced, bp->kmalloced_dma);

	dfx_bus_uninit(dev);

	dfx_get_bars(bdev, &bar_start, &bar_len);
	if (dfx_use_mmio) {
		iounmap(bp->base.mem);
		release_mem_region(bar_start, bar_len);
	} else
		release_region(bar_start, bar_len);

	if (dfx_bus_pci)
		pci_disable_device(to_pci_dev(bdev));

	free_netdev(dev);
}


static int __devinit __maybe_unused dfx_dev_register(struct device *);
static int __devexit __maybe_unused dfx_dev_unregister(struct device *);

#ifdef CONFIG_PCI
static int __devinit dfx_pci_register(struct pci_dev *,
				      const struct pci_device_id *);
static void __devexit dfx_pci_unregister(struct pci_dev *);

static DEFINE_PCI_DEVICE_TABLE(dfx_pci_table) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_DEC, PCI_DEVICE_ID_DEC_FDDI) },
	{ }
};
MODULE_DEVICE_TABLE(pci, dfx_pci_table);

static struct pci_driver dfx_pci_driver = {
	.name		= "defxx",
	.id_table	= dfx_pci_table,
	.probe		= dfx_pci_register,
	.remove		= __devexit_p(dfx_pci_unregister),
};

static __devinit int dfx_pci_register(struct pci_dev *pdev,
				      const struct pci_device_id *ent)
{
	return dfx_register(&pdev->dev);
}

static void __devexit dfx_pci_unregister(struct pci_dev *pdev)
{
	dfx_unregister(&pdev->dev);
}
#endif /* CONFIG_PCI */

#ifdef CONFIG_EISA
static struct eisa_device_id dfx_eisa_table[] = {
        { "DEC3001", DEFEA_PROD_ID_1 },
        { "DEC3002", DEFEA_PROD_ID_2 },
        { "DEC3003", DEFEA_PROD_ID_3 },
        { "DEC3004", DEFEA_PROD_ID_4 },
        { }
};
MODULE_DEVICE_TABLE(eisa, dfx_eisa_table);

static struct eisa_driver dfx_eisa_driver = {
	.id_table	= dfx_eisa_table,
	.driver		= {
		.name	= "defxx",
		.bus	= &eisa_bus_type,
		.probe	= dfx_dev_register,
		.remove	= __devexit_p(dfx_dev_unregister),
	},
};
#endif /* CONFIG_EISA */

#ifdef CONFIG_TC
static struct tc_device_id const dfx_tc_table[] = {
	{ "DEC     ", "PMAF-FA " },
	{ "DEC     ", "PMAF-FD " },
	{ "DEC     ", "PMAF-FS " },
	{ "DEC     ", "PMAF-FU " },
	{ }
};
MODULE_DEVICE_TABLE(tc, dfx_tc_table);

static struct tc_driver dfx_tc_driver = {
	.id_table	= dfx_tc_table,
	.driver		= {
		.name	= "defxx",
		.bus	= &tc_bus_type,
		.probe	= dfx_dev_register,
		.remove	= __devexit_p(dfx_dev_unregister),
	},
};
#endif /* CONFIG_TC */

static int __devinit __maybe_unused dfx_dev_register(struct device *dev)
{
	int status;

	status = dfx_register(dev);
	if (!status)
		get_device(dev);
	return status;
}

static int __devexit __maybe_unused dfx_dev_unregister(struct device *dev)
{
	put_device(dev);
	dfx_unregister(dev);
	return 0;
}


static int __devinit dfx_init(void)
{
	int status;

	status = pci_register_driver(&dfx_pci_driver);
	if (!status)
		status = eisa_driver_register(&dfx_eisa_driver);
	if (!status)
		status = tc_register_driver(&dfx_tc_driver);
	return status;
}

static void __devexit dfx_cleanup(void)
{
	tc_unregister_driver(&dfx_tc_driver);
	eisa_driver_unregister(&dfx_eisa_driver);
	pci_unregister_driver(&dfx_pci_driver);
}

module_init(dfx_init);
module_exit(dfx_cleanup);
MODULE_AUTHOR("Lawrence V. Stefani");
MODULE_DESCRIPTION("DEC FDDIcontroller TC/EISA/PCI (DEFTA/DEFEA/DEFPA) driver "
		   DRV_VERSION " " DRV_RELDATE);
MODULE_LICENSE("GPL");
