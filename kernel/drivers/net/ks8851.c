

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>

#include <linux/spi/spi.h>

#include "ks8851.h"

struct ks8851_rxctrl {
	u16	mchash[4];
	u16	rxcr1;
	u16	rxcr2;
};

union ks8851_tx_hdr {
	u8	txb[6];
	__le16	txw[3];
};

struct ks8851_net {
	struct net_device	*netdev;
	struct spi_device	*spidev;
	struct mutex		lock;
	spinlock_t		statelock;

	union ks8851_tx_hdr	txh ____cacheline_aligned;
	u8			rxd[8];
	u8			txd[8];

	u32			msg_enable ____cacheline_aligned;
	u16			tx_space;
	u8			fid;

	u16			rc_ier;
	u16			rc_rxqcr;
	u16			rc_ccr;
	u16			eeprom_size;

	struct mii_if_info	mii;
	struct ks8851_rxctrl	rxctrl;

	struct work_struct	tx_work;
	struct work_struct	irq_work;
	struct work_struct	rxctrl_work;

	struct sk_buff_head	txq;

	struct spi_message	spi_msg1;
	struct spi_message	spi_msg2;
	struct spi_transfer	spi_xfer1;
	struct spi_transfer	spi_xfer2[2];
};

static int msg_enable;

/* shift for byte-enable data */
#define BYTE_EN(_x)	((_x) << 2)

/* turn register number and byte-enable mask into data for start of packet */
#define MK_OP(_byteen, _reg) (BYTE_EN(_byteen) | (_reg)  << (8+2) | (_reg) >> 6)


static void ks8851_wrreg16(struct ks8851_net *ks, unsigned reg, unsigned val)
{
	struct spi_transfer *xfer = &ks->spi_xfer1;
	struct spi_message *msg = &ks->spi_msg1;
	__le16 txb[2];
	int ret;

	txb[0] = cpu_to_le16(MK_OP(reg & 2 ? 0xC : 0x03, reg) | KS_SPIOP_WR);
	txb[1] = cpu_to_le16(val);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 4;

	ret = spi_sync(ks->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "spi_sync() failed\n");
}

static void ks8851_wrreg8(struct ks8851_net *ks, unsigned reg, unsigned val)
{
	struct spi_transfer *xfer = &ks->spi_xfer1;
	struct spi_message *msg = &ks->spi_msg1;
	__le16 txb[2];
	int ret;
	int bit;

	bit = 1 << (reg & 3);

	txb[0] = cpu_to_le16(MK_OP(bit, reg) | KS_SPIOP_WR);
	txb[1] = val;

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 3;

	ret = spi_sync(ks->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "spi_sync() failed\n");
}

static inline bool ks8851_rx_1msg(struct ks8851_net *ks)
{
	return true;
}

static void ks8851_rdreg(struct ks8851_net *ks, unsigned op,
			 u8 *rxb, unsigned rxl)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	__le16 *txb = (__le16 *)ks->txd;
	u8 *trx = ks->rxd;
	int ret;

	txb[0] = cpu_to_le16(op | KS_SPIOP_RD);

	if (ks8851_rx_1msg(ks)) {
		msg = &ks->spi_msg1;
		xfer = &ks->spi_xfer1;

		xfer->tx_buf = txb;
		xfer->rx_buf = trx;
		xfer->len = rxl + 2;
	} else {
		msg = &ks->spi_msg2;
		xfer = ks->spi_xfer2;

		xfer->tx_buf = txb;
		xfer->rx_buf = NULL;
		xfer->len = 2;

		xfer++;
		xfer->tx_buf = NULL;
		xfer->rx_buf = trx;
		xfer->len = rxl;
	}

	ret = spi_sync(ks->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "read: spi_sync() failed\n");
	else if (ks8851_rx_1msg(ks))
		memcpy(rxb, trx + 2, rxl);
	else
		memcpy(rxb, trx, rxl);
}

static unsigned ks8851_rdreg8(struct ks8851_net *ks, unsigned reg)
{
	u8 rxb[1];

	ks8851_rdreg(ks, MK_OP(1 << (reg & 3), reg), rxb, 1);
	return rxb[0];
}

static unsigned ks8851_rdreg16(struct ks8851_net *ks, unsigned reg)
{
	__le16 rx = 0;

	ks8851_rdreg(ks, MK_OP(reg & 2 ? 0xC : 0x3, reg), (u8 *)&rx, 2);
	return le16_to_cpu(rx);
}

static unsigned ks8851_rdreg32(struct ks8851_net *ks, unsigned reg)
{
	__le32 rx = 0;

	WARN_ON(reg & 3);

	ks8851_rdreg(ks, MK_OP(0xf, reg), (u8 *)&rx, 4);
	return le32_to_cpu(rx);
}

static void ks8851_soft_reset(struct ks8851_net *ks, unsigned op)
{
	ks8851_wrreg16(ks, KS_GRR, op);
	mdelay(1);	/* wait a short time to effect reset */
	ks8851_wrreg16(ks, KS_GRR, 0);
	mdelay(1);	/* wait for condition to clear */
}

static int ks8851_write_mac_addr(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	int i;

	mutex_lock(&ks->lock);

	for (i = 0; i < ETH_ALEN; i++)
		ks8851_wrreg8(ks, KS_MAR(i), dev->dev_addr[i]);

	mutex_unlock(&ks->lock);

	return 0;
}

static void ks8851_init_mac(struct ks8851_net *ks)
{
	struct net_device *dev = ks->netdev;

	random_ether_addr(dev->dev_addr);
	ks8851_write_mac_addr(dev);
}

static irqreturn_t ks8851_irq(int irq, void *pw)
{
	struct ks8851_net *ks = pw;

	disable_irq_nosync(irq);
	schedule_work(&ks->irq_work);
	return IRQ_HANDLED;
}

static void ks8851_rdfifo(struct ks8851_net *ks, u8 *buff, unsigned len)
{
	struct spi_transfer *xfer = ks->spi_xfer2;
	struct spi_message *msg = &ks->spi_msg2;
	u8 txb[1];
	int ret;

	netif_dbg(ks, rx_status, ks->netdev,
		  "%s: %d@%p\n", __func__, len, buff);

	/* set the operation we're issuing */
	txb[0] = KS_SPIOP_RXFIFO;

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 1;

	xfer++;
	xfer->rx_buf = buff;
	xfer->tx_buf = NULL;
	xfer->len = len;

	ret = spi_sync(ks->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "%s: spi_sync() failed\n", __func__);
}

static void ks8851_dbg_dumpkkt(struct ks8851_net *ks, u8 *rxpkt)
{
	netdev_dbg(ks->netdev,
		   "pkt %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
		   rxpkt[4], rxpkt[5], rxpkt[6], rxpkt[7],
		   rxpkt[8], rxpkt[9], rxpkt[10], rxpkt[11],
		   rxpkt[12], rxpkt[13], rxpkt[14], rxpkt[15]);
}

static void ks8851_rx_pkts(struct ks8851_net *ks)
{
	struct sk_buff *skb;
	unsigned rxfc;
	unsigned rxlen;
	unsigned rxstat;
	u32 rxh;
	u8 *rxpkt;

	rxfc = ks8851_rdreg8(ks, KS_RXFC);

	netif_dbg(ks, rx_status, ks->netdev,
		  "%s: %d packets\n", __func__, rxfc);

	/* Currently we're issuing a read per packet, but we could possibly
	 * improve the code by issuing a single read, getting the receive
	 * header, allocating the packet and then reading the packet data
	 * out in one go.
	 *
	 * This form of operation would require us to hold the SPI bus'
	 * chipselect low during the entie transaction to avoid any
	 * reset to the data stream comming from the chip.
	 */

	for (; rxfc != 0; rxfc--) {
		rxh = ks8851_rdreg32(ks, KS_RXFHSR);
		rxstat = rxh & 0xffff;
		rxlen = rxh >> 16;

		netif_dbg(ks, rx_status, ks->netdev,
			  "rx: stat 0x%04x, len 0x%04x\n", rxstat, rxlen);

		/* the length of the packet includes the 32bit CRC */

		/* set dma read address */
		ks8851_wrreg16(ks, KS_RXFDPR, RXFDPR_RXFPAI | 0x00);

		/* start the packet dma process, and set auto-dequeue rx */
		ks8851_wrreg16(ks, KS_RXQCR,
			       ks->rc_rxqcr | RXQCR_SDA | RXQCR_ADRFE);

		if (rxlen > 0) {
			skb = netdev_alloc_skb(ks->netdev, rxlen + 2 + 8);
			if (!skb) {
				/* todo - dump frame and move on */
			}

			/* two bytes to ensure ip is aligned, and four bytes
			 * for the status header and 4 bytes of garbage */
			skb_reserve(skb, 2 + 4 + 4);

			rxpkt = skb_put(skb, rxlen - 4) - 8;

			/* align the packet length to 4 bytes, and add 4 bytes
			 * as we're getting the rx status header as well */
			ks8851_rdfifo(ks, rxpkt, ALIGN(rxlen, 4) + 8);

			if (netif_msg_pktdata(ks))
				ks8851_dbg_dumpkkt(ks, rxpkt);

			skb->protocol = eth_type_trans(skb, ks->netdev);
			netif_rx(skb);

			ks->netdev->stats.rx_packets++;
			ks->netdev->stats.rx_bytes += rxlen - 4;
		}

		ks8851_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr);
	}
}

static void ks8851_irq_work(struct work_struct *work)
{
	struct ks8851_net *ks = container_of(work, struct ks8851_net, irq_work);
	unsigned status;
	unsigned handled = 0;

	mutex_lock(&ks->lock);

	status = ks8851_rdreg16(ks, KS_ISR);

	netif_dbg(ks, intr, ks->netdev,
		  "%s: status 0x%04x\n", __func__, status);

	if (status & IRQ_LCI) {
		/* should do something about checking link status */
		handled |= IRQ_LCI;
	}

	if (status & IRQ_LDI) {
		u16 pmecr = ks8851_rdreg16(ks, KS_PMECR);
		pmecr &= ~PMECR_WKEVT_MASK;
		ks8851_wrreg16(ks, KS_PMECR, pmecr | PMECR_WKEVT_LINK);

		handled |= IRQ_LDI;
	}

	if (status & IRQ_RXPSI)
		handled |= IRQ_RXPSI;

	if (status & IRQ_TXI) {
		handled |= IRQ_TXI;

		/* no lock here, tx queue should have been stopped */

		/* update our idea of how much tx space is available to the
		 * system */
		ks->tx_space = ks8851_rdreg16(ks, KS_TXMIR);

		netif_dbg(ks, intr, ks->netdev,
			  "%s: txspace %d\n", __func__, ks->tx_space);
	}

	if (status & IRQ_RXI)
		handled |= IRQ_RXI;

	if (status & IRQ_SPIBEI) {
		dev_err(&ks->spidev->dev, "%s: spi bus error\n", __func__);
		handled |= IRQ_SPIBEI;
	}

	ks8851_wrreg16(ks, KS_ISR, handled);

	if (status & IRQ_RXI) {
		/* the datasheet says to disable the rx interrupt during
		 * packet read-out, however we're masking the interrupt
		 * from the device so do not bother masking just the RX
		 * from the device. */

		ks8851_rx_pkts(ks);
	}

	/* if something stopped the rx process, probably due to wanting
	 * to change the rx settings, then do something about restarting
	 * it. */
	if (status & IRQ_RXPSI) {
		struct ks8851_rxctrl *rxc = &ks->rxctrl;

		/* update the multicast hash table */
		ks8851_wrreg16(ks, KS_MAHTR0, rxc->mchash[0]);
		ks8851_wrreg16(ks, KS_MAHTR1, rxc->mchash[1]);
		ks8851_wrreg16(ks, KS_MAHTR2, rxc->mchash[2]);
		ks8851_wrreg16(ks, KS_MAHTR3, rxc->mchash[3]);

		ks8851_wrreg16(ks, KS_RXCR2, rxc->rxcr2);
		ks8851_wrreg16(ks, KS_RXCR1, rxc->rxcr1);
	}

	mutex_unlock(&ks->lock);

	if (status & IRQ_TXI)
		netif_wake_queue(ks->netdev);

	enable_irq(ks->netdev->irq);
}

static inline unsigned calc_txlen(unsigned len)
{
	return ALIGN(len + 4, 4);
}

static void ks8851_wrpkt(struct ks8851_net *ks, struct sk_buff *txp, bool irq)
{
	struct spi_transfer *xfer = ks->spi_xfer2;
	struct spi_message *msg = &ks->spi_msg2;
	unsigned fid = 0;
	int ret;

	netif_dbg(ks, tx_queued, ks->netdev, "%s: skb %p, %d@%p, irq %d\n",
		  __func__, txp, txp->len, txp->data, irq);

	fid = ks->fid++;
	fid &= TXFR_TXFID_MASK;

	if (irq)
		fid |= TXFR_TXIC;	/* irq on completion */

	/* start header at txb[1] to align txw entries */
	ks->txh.txb[1] = KS_SPIOP_TXFIFO;
	ks->txh.txw[1] = cpu_to_le16(fid);
	ks->txh.txw[2] = cpu_to_le16(txp->len);

	xfer->tx_buf = &ks->txh.txb[1];
	xfer->rx_buf = NULL;
	xfer->len = 5;

	xfer++;
	xfer->tx_buf = txp->data;
	xfer->rx_buf = NULL;
	xfer->len = ALIGN(txp->len, 4);

	ret = spi_sync(ks->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "%s: spi_sync() failed\n", __func__);
}

static void ks8851_done_tx(struct ks8851_net *ks, struct sk_buff *txb)
{
	struct net_device *dev = ks->netdev;

	dev->stats.tx_bytes += txb->len;
	dev->stats.tx_packets++;

	dev_kfree_skb(txb);
}

static void ks8851_tx_work(struct work_struct *work)
{
	struct ks8851_net *ks = container_of(work, struct ks8851_net, tx_work);
	struct sk_buff *txb;
	bool last = skb_queue_empty(&ks->txq);

	mutex_lock(&ks->lock);

	while (!last) {
		txb = skb_dequeue(&ks->txq);
		last = skb_queue_empty(&ks->txq);

		if (txb != NULL) {
			ks8851_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr | RXQCR_SDA);
			ks8851_wrpkt(ks, txb, last);
			ks8851_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr);
			ks8851_wrreg16(ks, KS_TXQCR, TXQCR_METFE);

			ks8851_done_tx(ks, txb);
		}
	}

	mutex_unlock(&ks->lock);
}

static void ks8851_set_powermode(struct ks8851_net *ks, unsigned pwrmode)
{
	unsigned pmecr;

	netif_dbg(ks, hw, ks->netdev, "setting power mode %d\n", pwrmode);

	pmecr = ks8851_rdreg16(ks, KS_PMECR);
	pmecr &= ~PMECR_PM_MASK;
	pmecr |= pwrmode;

	ks8851_wrreg16(ks, KS_PMECR, pmecr);
}

static int ks8851_net_open(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);

	/* lock the card, even if we may not actually be doing anything
	 * else at the moment */
	mutex_lock(&ks->lock);

	netif_dbg(ks, ifup, ks->netdev, "opening\n");

	/* bring chip out of any power saving mode it was in */
	ks8851_set_powermode(ks, PMECR_PM_NORMAL);

	/* issue a soft reset to the RX/TX QMU to put it into a known
	 * state. */
	ks8851_soft_reset(ks, GRR_QMU);

	/* setup transmission parameters */

	ks8851_wrreg16(ks, KS_TXCR, (TXCR_TXE | /* enable transmit process */
				     TXCR_TXPE | /* pad to min length */
				     TXCR_TXCRC | /* add CRC */
				     TXCR_TXFCE)); /* enable flow control */

	/* auto-increment tx data, reset tx pointer */
	ks8851_wrreg16(ks, KS_TXFDPR, TXFDPR_TXFPAI);

	/* setup receiver control */

	ks8851_wrreg16(ks, KS_RXCR1, (RXCR1_RXPAFMA | /*  from mac filter */
				      RXCR1_RXFCE | /* enable flow control */
				      RXCR1_RXBE | /* broadcast enable */
				      RXCR1_RXUE | /* unicast enable */
				      RXCR1_RXE)); /* enable rx block */

	/* transfer entire frames out in one go */
	ks8851_wrreg16(ks, KS_RXCR2, RXCR2_SRDBL_FRAME);

	/* set receive counter timeouts */
	ks8851_wrreg16(ks, KS_RXDTTR, 1000); /* 1ms after first frame to IRQ */
	ks8851_wrreg16(ks, KS_RXDBCTR, 4096); /* >4Kbytes in buffer to IRQ */
	ks8851_wrreg16(ks, KS_RXFCTR, 10);  /* 10 frames to IRQ */

	ks->rc_rxqcr = (RXQCR_RXFCTE |  /* IRQ on frame count exceeded */
			RXQCR_RXDBCTE | /* IRQ on byte count exceeded */
			RXQCR_RXDTTE);  /* IRQ on time exceeded */

	ks8851_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr);

	/* clear then enable interrupts */

#define STD_IRQ (IRQ_LCI |	/* Link Change */	\
		 IRQ_TXI |	/* TX done */		\
		 IRQ_RXI |	/* RX done */		\
		 IRQ_SPIBEI |	/* SPI bus error */	\
		 IRQ_TXPSI |	/* TX process stop */	\
		 IRQ_RXPSI)	/* RX process stop */

	ks->rc_ier = STD_IRQ;
	ks8851_wrreg16(ks, KS_ISR, STD_IRQ);
	ks8851_wrreg16(ks, KS_IER, STD_IRQ);

	netif_start_queue(ks->netdev);

	netif_dbg(ks, ifup, ks->netdev, "network device up\n");

	mutex_unlock(&ks->lock);
	return 0;
}

static int ks8851_net_stop(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);

	netif_info(ks, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	mutex_lock(&ks->lock);

	/* stop any outstanding work */
	flush_work(&ks->irq_work);
	flush_work(&ks->tx_work);
	flush_work(&ks->rxctrl_work);

	/* turn off the IRQs and ack any outstanding */
	ks8851_wrreg16(ks, KS_IER, 0x0000);
	ks8851_wrreg16(ks, KS_ISR, 0xffff);

	/* shutdown RX process */
	ks8851_wrreg16(ks, KS_RXCR1, 0x0000);

	/* shutdown TX process */
	ks8851_wrreg16(ks, KS_TXCR, 0x0000);

	/* set powermode to soft power down to save power */
	ks8851_set_powermode(ks, PMECR_PM_SOFTDOWN);

	/* ensure any queued tx buffers are dumped */
	while (!skb_queue_empty(&ks->txq)) {
		struct sk_buff *txb = skb_dequeue(&ks->txq);

		netif_dbg(ks, ifdown, ks->netdev,
			  "%s: freeing txb %p\n", __func__, txb);

		dev_kfree_skb(txb);
	}

	mutex_unlock(&ks->lock);
	return 0;
}

static netdev_tx_t ks8851_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	unsigned needed = calc_txlen(skb->len);
	netdev_tx_t ret = NETDEV_TX_OK;

	netif_dbg(ks, tx_queued, ks->netdev,
		  "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);

	spin_lock(&ks->statelock);

	if (needed > ks->tx_space) {
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	} else {
		ks->tx_space -= needed;
		skb_queue_tail(&ks->txq, skb);
	}

	spin_unlock(&ks->statelock);
	schedule_work(&ks->tx_work);

	return ret;
}

static void ks8851_rxctrl_work(struct work_struct *work)
{
	struct ks8851_net *ks = container_of(work, struct ks8851_net, rxctrl_work);

	mutex_lock(&ks->lock);

	/* need to shutdown RXQ before modifying filter parameters */
	ks8851_wrreg16(ks, KS_RXCR1, 0x00);

	mutex_unlock(&ks->lock);
}

static void ks8851_set_rx_mode(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	struct ks8851_rxctrl rxctrl;

	memset(&rxctrl, 0, sizeof(rxctrl));

	if (dev->flags & IFF_PROMISC) {
		/* interface to receive everything */

		rxctrl.rxcr1 = RXCR1_RXAE | RXCR1_RXINVF;
	} else if (dev->flags & IFF_ALLMULTI) {
		/* accept all multicast packets */

		rxctrl.rxcr1 = (RXCR1_RXME | RXCR1_RXAE |
				RXCR1_RXPAFMA | RXCR1_RXMAFMA);
	} else if (dev->flags & IFF_MULTICAST && !netdev_mc_empty(dev)) {
		struct netdev_hw_addr *ha;
		u32 crc;

		/* accept some multicast */

		netdev_for_each_mc_addr(ha, dev) {
			crc = ether_crc(ETH_ALEN, ha->addr);
			crc >>= (32 - 6);  /* get top six bits */

			rxctrl.mchash[crc >> 4] |= (1 << (crc & 0xf));
		}

		rxctrl.rxcr1 = RXCR1_RXME | RXCR1_RXPAFMA;
	} else {
		/* just accept broadcast / unicast */
		rxctrl.rxcr1 = RXCR1_RXPAFMA;
	}

	rxctrl.rxcr1 |= (RXCR1_RXUE | /* unicast enable */
			 RXCR1_RXBE | /* broadcast enable */
			 RXCR1_RXE | /* RX process enable */
			 RXCR1_RXFCE); /* enable flow control */

	rxctrl.rxcr2 |= RXCR2_SRDBL_FRAME;

	/* schedule work to do the actual set of the data if needed */

	spin_lock(&ks->statelock);

	if (memcmp(&rxctrl, &ks->rxctrl, sizeof(rxctrl)) != 0) {
		memcpy(&ks->rxctrl, &rxctrl, sizeof(ks->rxctrl));
		schedule_work(&ks->rxctrl_work);
	}

	spin_unlock(&ks->statelock);
}

static int ks8851_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sa = addr;

	if (netif_running(dev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);
	return ks8851_write_mac_addr(dev);
}

static int ks8851_net_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	struct ks8851_net *ks = netdev_priv(dev);

	if (!netif_running(dev))
		return -EINVAL;

	return generic_mii_ioctl(&ks->mii, if_mii(req), cmd, NULL);
}

static const struct net_device_ops ks8851_netdev_ops = {
	.ndo_open		= ks8851_net_open,
	.ndo_stop		= ks8851_net_stop,
	.ndo_do_ioctl		= ks8851_net_ioctl,
	.ndo_start_xmit		= ks8851_start_xmit,
	.ndo_set_mac_address	= ks8851_set_mac_address,
	.ndo_set_rx_mode	= ks8851_set_rx_mode,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

/* Companion eeprom access */

enum {	/* EEPROM programming states */
	EEPROM_CONTROL,
	EEPROM_ADDRESS,
	EEPROM_DATA,
	EEPROM_COMPLETE
};

unsigned int ks8851_eeprom_read(struct net_device *dev, unsigned int addr)
{
	struct ks8851_net *ks = netdev_priv(dev);
	int eepcr;
	int ctrl = EEPROM_OP_READ;
	int state = EEPROM_CONTROL;
	int bit_count = EEPROM_OP_LEN - 1;
	unsigned int data = 0;
	int dummy;
	unsigned int addr_len;

	addr_len = (ks->eeprom_size == 128) ? 6 : 8;

	/* start transaction: chip select high, authorize write */
	mutex_lock(&ks->lock);
	eepcr = EEPCR_EESA | EEPCR_EESRWA;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	eepcr |= EEPCR_EECS;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	mutex_unlock(&ks->lock);

	while (state != EEPROM_COMPLETE) {
		/* falling clock period starts... */
		/* set EED_IO pin for control and address */
		eepcr &= ~EEPCR_EEDO;
		switch (state) {
		case EEPROM_CONTROL:
			eepcr |= ((ctrl >> bit_count) & 1) << 2;
			if (bit_count-- <= 0) {
				bit_count = addr_len - 1;
				state = EEPROM_ADDRESS;
			}
			break;
		case EEPROM_ADDRESS:
			eepcr |= ((addr >> bit_count) & 1) << 2;
			bit_count--;
			break;
		case EEPROM_DATA:
			/* Change to receive mode */
			eepcr &= ~EEPCR_EESRWA;
			break;
		}

		/* lower clock  */
		eepcr &= ~EEPCR_EESCK;

		mutex_lock(&ks->lock);
		ks8851_wrreg16(ks, KS_EEPCR, eepcr);
		mutex_unlock(&ks->lock);

		/* waitread period / 2 */
		udelay(EEPROM_SK_PERIOD / 2);

		/* rising clock period starts... */

		/* raise clock */
		mutex_lock(&ks->lock);
		eepcr |= EEPCR_EESCK;
		ks8851_wrreg16(ks, KS_EEPCR, eepcr);
		mutex_unlock(&ks->lock);

		/* Manage read */
		switch (state) {
		case EEPROM_ADDRESS:
			if (bit_count < 0) {
				bit_count = EEPROM_DATA_LEN - 1;
				state = EEPROM_DATA;
			}
			break;
		case EEPROM_DATA:
			mutex_lock(&ks->lock);
			dummy = ks8851_rdreg16(ks, KS_EEPCR);
			mutex_unlock(&ks->lock);
			data |= ((dummy >> EEPCR_EESB_OFFSET) & 1) << bit_count;
			if (bit_count-- <= 0)
				state = EEPROM_COMPLETE;
			break;
		}

		/* wait period / 2 */
		udelay(EEPROM_SK_PERIOD / 2);
	}

	/* close transaction */
	mutex_lock(&ks->lock);
	eepcr &= ~EEPCR_EECS;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	eepcr = 0;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	mutex_unlock(&ks->lock);

	return data;
}

void ks8851_eeprom_write(struct net_device *dev, unsigned int op,
					unsigned int addr, unsigned int data)
{
	struct ks8851_net *ks = netdev_priv(dev);
	int eepcr;
	int state = EEPROM_CONTROL;
	int bit_count = EEPROM_OP_LEN - 1;
	unsigned int addr_len;

	addr_len = (ks->eeprom_size == 128) ? 6 : 8;

	switch (op) {
	case EEPROM_OP_EWEN:
		addr = 0x30;
	break;
	case EEPROM_OP_EWDS:
		addr = 0;
		break;
	}

	/* start transaction: chip select high, authorize write */
	mutex_lock(&ks->lock);
	eepcr = EEPCR_EESA | EEPCR_EESRWA;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	eepcr |= EEPCR_EECS;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	mutex_unlock(&ks->lock);

	while (state != EEPROM_COMPLETE) {
		/* falling clock period starts... */
		/* set EED_IO pin for control and address */
		eepcr &= ~EEPCR_EEDO;
		switch (state) {
		case EEPROM_CONTROL:
			eepcr |= ((op >> bit_count) & 1) << 2;
			if (bit_count-- <= 0) {
				bit_count = addr_len - 1;
				state = EEPROM_ADDRESS;
			}
			break;
		case EEPROM_ADDRESS:
			eepcr |= ((addr >> bit_count) & 1) << 2;
			if (bit_count-- <= 0) {
				if (op == EEPROM_OP_WRITE) {
					bit_count = EEPROM_DATA_LEN - 1;
					state = EEPROM_DATA;
				} else {
					state = EEPROM_COMPLETE;
				}
			}
			break;
		case EEPROM_DATA:
			eepcr |= ((data >> bit_count) & 1) << 2;
			if (bit_count-- <= 0)
				state = EEPROM_COMPLETE;
			break;
		}

		/* lower clock  */
		eepcr &= ~EEPCR_EESCK;

		mutex_lock(&ks->lock);
		ks8851_wrreg16(ks, KS_EEPCR, eepcr);
		mutex_unlock(&ks->lock);

		/* wait period / 2 */
		udelay(EEPROM_SK_PERIOD / 2);

		/* rising clock period starts... */

		/* raise clock */
		eepcr |= EEPCR_EESCK;
		mutex_lock(&ks->lock);
		ks8851_wrreg16(ks, KS_EEPCR, eepcr);
		mutex_unlock(&ks->lock);

		/* wait period / 2 */
		udelay(EEPROM_SK_PERIOD / 2);
	}

	/* close transaction */
	mutex_lock(&ks->lock);
	eepcr &= ~EEPCR_EECS;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	eepcr = 0;
	ks8851_wrreg16(ks, KS_EEPCR, eepcr);
	mutex_unlock(&ks->lock);

}

/* ethtool support */

static void ks8851_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *di)
{
	strlcpy(di->driver, "KS8851", sizeof(di->driver));
	strlcpy(di->version, "1.00", sizeof(di->version));
	strlcpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static u32 ks8851_get_msglevel(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return ks->msg_enable;
}

static void ks8851_set_msglevel(struct net_device *dev, u32 to)
{
	struct ks8851_net *ks = netdev_priv(dev);
	ks->msg_enable = to;
}

static int ks8851_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return mii_ethtool_gset(&ks->mii, cmd);
}

static int ks8851_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return mii_ethtool_sset(&ks->mii, cmd);
}

static u32 ks8851_get_link(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return mii_link_ok(&ks->mii);
}

static int ks8851_nway_reset(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return mii_nway_restart(&ks->mii);
}

static int ks8851_get_eeprom_len(struct net_device *dev)
{
	struct ks8851_net *ks = netdev_priv(dev);
	return ks->eeprom_size;
}

static int ks8851_get_eeprom(struct net_device *dev,
			    struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct ks8851_net *ks = netdev_priv(dev);
	u16 *eeprom_buff;
	int first_word;
	int last_word;
	int ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EINVAL;

	if (eeprom->len > ks->eeprom_size)
		return -EINVAL;

	eeprom->magic = ks8851_rdreg16(ks, KS_CIDER);

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;

	eeprom_buff = kmalloc(sizeof(u16) *
			(last_word - first_word + 1), GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	for (i = 0; i < last_word - first_word + 1; i++)
		eeprom_buff[i] = ks8851_eeprom_read(dev, first_word + 1);

	/* Device's eeprom is little-endian, word addressable */
	for (i = 0; i < last_word - first_word + 1; i++)
		le16_to_cpus(&eeprom_buff[i]);

	memcpy(bytes, (u8 *)eeprom_buff + (eeprom->offset & 1), eeprom->len);
	kfree(eeprom_buff);

	return ret_val;
}

static int ks8851_set_eeprom(struct net_device *dev,
			    struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct ks8851_net *ks = netdev_priv(dev);
	u16 *eeprom_buff;
	void *ptr;
	int max_len;
	int first_word;
	int last_word;
	int ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EOPNOTSUPP;

	if (eeprom->len > ks->eeprom_size)
		return -EINVAL;

	if (eeprom->magic != ks8851_rdreg16(ks, KS_CIDER))
		return -EFAULT;

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	max_len = (last_word - first_word + 1) * 2;
	eeprom_buff = kmalloc(max_len, GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	ptr = (void *)eeprom_buff;

	if (eeprom->offset & 1) {
		/* need read/modify/write of first changed EEPROM word */
		/* only the second byte of the word is being modified */
		eeprom_buff[0] = ks8851_eeprom_read(dev, first_word);
		ptr++;
	}
	if ((eeprom->offset + eeprom->len) & 1)
		/* need read/modify/write of last changed EEPROM word */
		/* only the first byte of the word is being modified */
		eeprom_buff[last_word - first_word] =
					ks8851_eeprom_read(dev, last_word);


	/* Device's eeprom is little-endian, word addressable */
	le16_to_cpus(&eeprom_buff[0]);
	le16_to_cpus(&eeprom_buff[last_word - first_word]);

	memcpy(ptr, bytes, eeprom->len);

	for (i = 0; i < last_word - first_word + 1; i++)
		eeprom_buff[i] = cpu_to_le16(eeprom_buff[i]);

	ks8851_eeprom_write(dev, EEPROM_OP_EWEN, 0, 0);

	for (i = 0; i < last_word - first_word + 1; i++) {
		ks8851_eeprom_write(dev, EEPROM_OP_WRITE, first_word + i,
							eeprom_buff[i]);
		mdelay(EEPROM_WRITE_TIME);
	}

	ks8851_eeprom_write(dev, EEPROM_OP_EWDS, 0, 0);

	kfree(eeprom_buff);
	return ret_val;
}

static const struct ethtool_ops ks8851_ethtool_ops = {
	.get_drvinfo	= ks8851_get_drvinfo,
	.get_msglevel	= ks8851_get_msglevel,
	.set_msglevel	= ks8851_set_msglevel,
	.get_settings	= ks8851_get_settings,
	.set_settings	= ks8851_set_settings,
	.get_link	= ks8851_get_link,
	.nway_reset	= ks8851_nway_reset,
	.get_eeprom_len	= ks8851_get_eeprom_len,
	.get_eeprom	= ks8851_get_eeprom,
	.set_eeprom	= ks8851_set_eeprom,
};

/* MII interface controls */

static int ks8851_phy_reg(int reg)
{
	switch (reg) {
	case MII_BMCR:
		return KS_P1MBCR;
	case MII_BMSR:
		return KS_P1MBSR;
	case MII_PHYSID1:
		return KS_PHY1ILR;
	case MII_PHYSID2:
		return KS_PHY1IHR;
	case MII_ADVERTISE:
		return KS_P1ANAR;
	case MII_LPA:
		return KS_P1ANLPR;
	}

	return 0x0;
}

static int ks8851_phy_read(struct net_device *dev, int phy_addr, int reg)
{
	struct ks8851_net *ks = netdev_priv(dev);
	int ksreg;
	int result;

	ksreg = ks8851_phy_reg(reg);
	if (!ksreg)
		return 0x0;	/* no error return allowed, so use zero */

	mutex_lock(&ks->lock);
	result = ks8851_rdreg16(ks, ksreg);
	mutex_unlock(&ks->lock);

	return result;
}

static void ks8851_phy_write(struct net_device *dev,
			     int phy, int reg, int value)
{
	struct ks8851_net *ks = netdev_priv(dev);
	int ksreg;

	ksreg = ks8851_phy_reg(reg);
	if (ksreg) {
		mutex_lock(&ks->lock);
		ks8851_wrreg16(ks, ksreg, value);
		mutex_unlock(&ks->lock);
	}
}

static int ks8851_read_selftest(struct ks8851_net *ks)
{
	unsigned both_done = MBIR_TXMBF | MBIR_RXMBF;
	int ret = 0;
	unsigned rd;

	rd = ks8851_rdreg16(ks, KS_MBIR);

	if ((rd & both_done) != both_done) {
		netdev_warn(ks->netdev, "Memory selftest not finished\n");
		return 0;
	}

	if (rd & MBIR_TXMBFA) {
		netdev_err(ks->netdev, "TX memory selftest fail\n");
		ret |= 1;
	}

	if (rd & MBIR_RXMBFA) {
		netdev_err(ks->netdev, "RX memory selftest fail\n");
		ret |= 2;
	}

	return 0;
}

/* driver bus management functions */

static int __devinit ks8851_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct ks8851_net *ks;
	int ret;

	ndev = alloc_etherdev(sizeof(struct ks8851_net));
	if (!ndev) {
		dev_err(&spi->dev, "failed to alloc ethernet device\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 8;

	ks = netdev_priv(ndev);

	ks->netdev = ndev;
	ks->spidev = spi;
	ks->tx_space = 6144;

	mutex_init(&ks->lock);
	spin_lock_init(&ks->statelock);

	INIT_WORK(&ks->tx_work, ks8851_tx_work);
	INIT_WORK(&ks->irq_work, ks8851_irq_work);
	INIT_WORK(&ks->rxctrl_work, ks8851_rxctrl_work);

	/* initialise pre-made spi transfer messages */

	spi_message_init(&ks->spi_msg1);
	spi_message_add_tail(&ks->spi_xfer1, &ks->spi_msg1);

	spi_message_init(&ks->spi_msg2);
	spi_message_add_tail(&ks->spi_xfer2[0], &ks->spi_msg2);
	spi_message_add_tail(&ks->spi_xfer2[1], &ks->spi_msg2);

	/* setup mii state */
	ks->mii.dev		= ndev;
	ks->mii.phy_id		= 1,
	ks->mii.phy_id_mask	= 1;
	ks->mii.reg_num_mask	= 0xf;
	ks->mii.mdio_read	= ks8851_phy_read;
	ks->mii.mdio_write	= ks8851_phy_write;

	dev_info(&spi->dev, "message enable is %d\n", msg_enable);

	/* set the default message enable */
	ks->msg_enable = netif_msg_init(msg_enable, (NETIF_MSG_DRV |
						     NETIF_MSG_PROBE |
						     NETIF_MSG_LINK));

	skb_queue_head_init(&ks->txq);

	SET_ETHTOOL_OPS(ndev, &ks8851_ethtool_ops);
	SET_NETDEV_DEV(ndev, &spi->dev);

	dev_set_drvdata(&spi->dev, ks);

	ndev->if_port = IF_PORT_100BASET;
	ndev->netdev_ops = &ks8851_netdev_ops;
	ndev->irq = spi->irq;

	/* issue a global soft reset to reset the device. */
	ks8851_soft_reset(ks, GRR_GSR);

	/* simple check for a valid chip being connected to the bus */

	if ((ks8851_rdreg16(ks, KS_CIDER) & ~CIDER_REV_MASK) != CIDER_ID) {
		dev_err(&spi->dev, "failed to read device ID\n");
		ret = -ENODEV;
		goto err_id;
	}

	/* cache the contents of the CCR register for EEPROM, etc. */
	ks->rc_ccr = ks8851_rdreg16(ks, KS_CCR);

	if (ks->rc_ccr & CCR_EEPROM)
		ks->eeprom_size = 128;
	else
		ks->eeprom_size = 0;

	ks8851_read_selftest(ks);
	ks8851_init_mac(ks);

	ret = request_irq(spi->irq, ks8851_irq, IRQF_TRIGGER_LOW,
			  ndev->name, ks);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to get irq\n");
		goto err_irq;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&spi->dev, "failed to register network device\n");
		goto err_netdev;
	}

	netdev_info(ndev, "revision %d, MAC %pM, IRQ %d\n",
		    CIDER_REV_GET(ks8851_rdreg16(ks, KS_CIDER)),
		    ndev->dev_addr, ndev->irq);

	return 0;


err_netdev:
	free_irq(ndev->irq, ndev);

err_id:
err_irq:
	free_netdev(ndev);
	return ret;
}

static int __devexit ks8851_remove(struct spi_device *spi)
{
	struct ks8851_net *priv = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(priv))
		dev_info(&spi->dev, "remove\n");

	unregister_netdev(priv->netdev);
	free_irq(spi->irq, priv);
	free_netdev(priv->netdev);

	return 0;
}

static struct spi_driver ks8851_driver = {
	.driver = {
		.name = "ks8851",
		.owner = THIS_MODULE,
	},
	.probe = ks8851_probe,
	.remove = __devexit_p(ks8851_remove),
};

static int __init ks8851_init(void)
{
	return spi_register_driver(&ks8851_driver);
}

static void __exit ks8851_exit(void)
{
	spi_unregister_driver(&ks8851_driver);
}

module_init(ks8851_init);
module_exit(ks8851_exit);

MODULE_DESCRIPTION("KS8851 Network driver");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");

module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, 31=all)");
MODULE_ALIAS("spi:ks8851");
