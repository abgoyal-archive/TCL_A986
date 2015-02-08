

#include "et131x_version.h"
#include "et131x_defs.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/pci.h>
#include <asm/system.h>

#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/ioport.h>

#include "et1310_phy.h"
#include "et1310_tx.h"
#include "et131x_adapter.h"
#include "et131x.h"

struct net_device_stats *et131x_stats(struct net_device *netdev);
int et131x_open(struct net_device *netdev);
int et131x_close(struct net_device *netdev);
int et131x_ioctl(struct net_device *netdev, struct ifreq *reqbuf, int cmd);
void et131x_multicast(struct net_device *netdev);
int et131x_tx(struct sk_buff *skb, struct net_device *netdev);
void et131x_tx_timeout(struct net_device *netdev);
int et131x_change_mtu(struct net_device *netdev, int new_mtu);
int et131x_set_mac_addr(struct net_device *netdev, void *new_mac);
void et131x_vlan_rx_register(struct net_device *netdev, struct vlan_group *grp);
void et131x_vlan_rx_add_vid(struct net_device *netdev, uint16_t vid);
void et131x_vlan_rx_kill_vid(struct net_device *netdev, uint16_t vid);

static const struct net_device_ops et131x_netdev_ops = {
	.ndo_open		= et131x_open,
	.ndo_stop		= et131x_close,
	.ndo_start_xmit		= et131x_tx,
	.ndo_set_multicast_list	= et131x_multicast,
	.ndo_tx_timeout		= et131x_tx_timeout,
	.ndo_change_mtu		= et131x_change_mtu,
	.ndo_set_mac_address	= et131x_set_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats		= et131x_stats,
	.ndo_do_ioctl		= et131x_ioctl,
};

struct net_device *et131x_device_alloc(void)
{
	struct net_device *netdev;

	/* Alloc net_device and adapter structs */
	netdev = alloc_etherdev(sizeof(struct et131x_adapter));

	if (netdev == NULL) {
		printk(KERN_ERR "et131x: Alloc of net_device struct failed\n");
		return NULL;
	}

	/* Setup the function registration table (and other data) for a
	 * net_device
	 */
	/* netdev->init               = &et131x_init; */
	/* netdev->set_config = &et131x_config; */
	netdev->watchdog_timeo = ET131X_TX_TIMEOUT;
	netdev->netdev_ops = &et131x_netdev_ops;

	/* netdev->ethtool_ops        = &et131x_ethtool_ops; */

	/* Poll? */
	/* netdev->poll               = &et131x_poll; */
	/* netdev->poll_controller    = &et131x_poll_controller; */
	return netdev;
}

struct net_device_stats *et131x_stats(struct net_device *netdev)
{
	struct et131x_adapter *adapter = netdev_priv(netdev);
	struct net_device_stats *stats = &adapter->net_stats;
	CE_STATS_t *devstat = &adapter->Stats;

	stats->rx_packets = devstat->ipackets;
	stats->tx_packets = devstat->opackets;
	stats->rx_errors = devstat->length_err + devstat->alignment_err +
	    devstat->crc_err + devstat->code_violations + devstat->other_errors;
	stats->tx_errors = devstat->max_pkt_error;
	stats->multicast = devstat->multircv;
	stats->collisions = devstat->collisions;

	stats->rx_length_errors = devstat->length_err;
	stats->rx_over_errors = devstat->rx_ov_flow;
	stats->rx_crc_errors = devstat->crc_err;

	/* NOTE: These stats don't have corresponding values in CE_STATS,
	 * so we're going to have to update these directly from within the
	 * TX/RX code
	 */
	/* stats->rx_bytes            = 20; devstat->; */
	/* stats->tx_bytes            = 20;  devstat->; */
	/* stats->rx_dropped          = devstat->; */
	/* stats->tx_dropped          = devstat->; */

	/*  NOTE: Not used, can't find analogous statistics */
	/* stats->rx_frame_errors     = devstat->; */
	/* stats->rx_fifo_errors      = devstat->; */
	/* stats->rx_missed_errors    = devstat->; */

	/* stats->tx_aborted_errors   = devstat->; */
	/* stats->tx_carrier_errors   = devstat->; */
	/* stats->tx_fifo_errors      = devstat->; */
	/* stats->tx_heartbeat_errors = devstat->; */
	/* stats->tx_window_errors    = devstat->; */
	return stats;
}

int et131x_open(struct net_device *netdev)
{
	int result = 0;
	struct et131x_adapter *adapter = netdev_priv(netdev);

	/* Start the timer to track NIC errors */
	add_timer(&adapter->ErrorTimer);

	/* Register our IRQ */
	result = request_irq(netdev->irq, et131x_isr, IRQF_SHARED,
					netdev->name, netdev);
	if (result) {
		dev_err(&adapter->pdev->dev, "c ould not register IRQ %d\n",
			netdev->irq);
		return result;
	}

	/* Enable the Tx and Rx DMA engines (if not already enabled) */
	et131x_rx_dma_enable(adapter);
	et131x_tx_dma_enable(adapter);

	/* Enable device interrupts */
	et131x_enable_interrupts(adapter);

	adapter->Flags |= fMP_ADAPTER_INTERRUPT_IN_USE;

	/* We're ready to move some data, so start the queue */
	netif_start_queue(netdev);
	return result;
}

int et131x_close(struct net_device *netdev)
{
	struct et131x_adapter *adapter = netdev_priv(netdev);

	/* First thing is to stop the queue */
	netif_stop_queue(netdev);

	/* Stop the Tx and Rx DMA engines */
	et131x_rx_dma_disable(adapter);
	et131x_tx_dma_disable(adapter);

	/* Disable device interrupts */
	et131x_disable_interrupts(adapter);

	/* Deregistering ISR */
	adapter->Flags &= ~fMP_ADAPTER_INTERRUPT_IN_USE;
	free_irq(netdev->irq, netdev);

	/* Stop the error timer */
	del_timer_sync(&adapter->ErrorTimer);
	return 0;
}

int et131x_ioctl_mii(struct net_device *netdev, struct ifreq *reqbuf, int cmd)
{
	int status = 0;
	struct et131x_adapter *etdev = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(reqbuf);

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = etdev->Stats.xcvr_addr;
		break;

	case SIOCGMIIREG:
		if (!capable(CAP_NET_ADMIN))
			status = -EPERM;
		else
			status = MiRead(etdev,
					data->reg_num, &data->val_out);
		break;

	case SIOCSMIIREG:
		if (!capable(CAP_NET_ADMIN))
			status = -EPERM;
		else
			status = MiWrite(etdev, data->reg_num,
					 data->val_in);
		break;

	default:
		status = -EOPNOTSUPP;
	}
	return status;
}

int et131x_ioctl(struct net_device *netdev, struct ifreq *reqbuf, int cmd)
{
	int status = 0;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		status = et131x_ioctl_mii(netdev, reqbuf, cmd);
		break;

	default:
		status = -EOPNOTSUPP;
	}
	return status;
}

int et131x_set_packet_filter(struct et131x_adapter *adapter)
{
	int status = 0;
	uint32_t filter = adapter->PacketFilter;
	u32 ctrl;
	u32 pf_ctrl;

	ctrl = readl(&adapter->regs->rxmac.ctrl);
	pf_ctrl = readl(&adapter->regs->rxmac.pf_ctrl);

	/* Default to disabled packet filtering.  Enable it in the individual
	 * case statements that require the device to filter something
	 */
	ctrl |= 0x04;

	/* Set us to be in promiscuous mode so we receive everything, this
	 * is also true when we get a packet filter of 0
	 */
	if ((filter & ET131X_PACKET_TYPE_PROMISCUOUS) || filter == 0)
		pf_ctrl &= ~7;	/* Clear filter bits */
	else {
		/*
		 * Set us up with Multicast packet filtering.  Three cases are
		 * possible - (1) we have a multi-cast list, (2) we receive ALL
		 * multicast entries or (3) we receive none.
		 */
		if (filter & ET131X_PACKET_TYPE_ALL_MULTICAST)
			pf_ctrl &= ~2;	/* Multicast filter bit */
		else {
			SetupDeviceForMulticast(adapter);
			pf_ctrl |= 2;
			ctrl &= ~0x04;
		}

		/* Set us up with Unicast packet filtering */
		if (filter & ET131X_PACKET_TYPE_DIRECTED) {
			SetupDeviceForUnicast(adapter);
			pf_ctrl |= 4;
			ctrl &= ~0x04;
		}

		/* Set us up with Broadcast packet filtering */
		if (filter & ET131X_PACKET_TYPE_BROADCAST) {
			pf_ctrl |= 1;	/* Broadcast filter bit */
			ctrl &= ~0x04;
		} else
			pf_ctrl &= ~1;

		/* Setup the receive mac configuration registers - Packet
		 * Filter control + the enable / disable for packet filter
		 * in the control reg.
		 */
		writel(pf_ctrl, &adapter->regs->rxmac.pf_ctrl);
		writel(ctrl, &adapter->regs->rxmac.ctrl);
	}
	return status;
}

void et131x_multicast(struct net_device *netdev)
{
	struct et131x_adapter *adapter = netdev_priv(netdev);
	uint32_t PacketFilter = 0;
	unsigned long flags;
	struct netdev_hw_addr *ha;
	int i;

	spin_lock_irqsave(&adapter->Lock, flags);

	/* Before we modify the platform-independent filter flags, store them
	 * locally. This allows us to determine if anything's changed and if
	 * we even need to bother the hardware
	 */
	PacketFilter = adapter->PacketFilter;

	/* Clear the 'multicast' flag locally; becuase we only have a single
	 * flag to check multicast, and multiple multicast addresses can be
	 * set, this is the easiest way to determine if more than one
	 * multicast address is being set.
	 */
	PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;

	/* Check the net_device flags and set the device independent flags
	 * accordingly
	 */

	if (netdev->flags & IFF_PROMISC)
		adapter->PacketFilter |= ET131X_PACKET_TYPE_PROMISCUOUS;
	else
		adapter->PacketFilter &= ~ET131X_PACKET_TYPE_PROMISCUOUS;

	if (netdev->flags & IFF_ALLMULTI)
		adapter->PacketFilter |= ET131X_PACKET_TYPE_ALL_MULTICAST;

	if (netdev_mc_count(netdev) > NIC_MAX_MCAST_LIST)
		adapter->PacketFilter |= ET131X_PACKET_TYPE_ALL_MULTICAST;

	if (netdev_mc_count(netdev) < 1) {
		adapter->PacketFilter &= ~ET131X_PACKET_TYPE_ALL_MULTICAST;
		adapter->PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;
	} else
		adapter->PacketFilter |= ET131X_PACKET_TYPE_MULTICAST;

	/* Set values in the private adapter struct */
	i = 0;
	netdev_for_each_mc_addr(ha, netdev) {
		if (i == NIC_MAX_MCAST_LIST)
			break;
		memcpy(adapter->MCList[i++], ha->addr, ETH_ALEN);
	}
	adapter->MCAddressCount = i;

	/* Are the new flags different from the previous ones? If not, then no
	 * action is required
	 *
	 * NOTE - This block will always update the MCList with the hardware,
	 *        even if the addresses aren't the same.
	 */
	if (PacketFilter != adapter->PacketFilter) {
		/* Call the device's filter function */
		et131x_set_packet_filter(adapter);
	}
	spin_unlock_irqrestore(&adapter->Lock, flags);
}

int et131x_tx(struct sk_buff *skb, struct net_device *netdev)
{
	int status = 0;

	/* Save the timestamp for the TX timeout watchdog */
	netdev->trans_start = jiffies;

	/* Call the device-specific data Tx routine */
	status = et131x_send_packets(skb, netdev);

	/* Check status and manage the netif queue if necessary */
	if (status != 0) {
		if (status == -ENOMEM) {
			/* Put the queue to sleep until resources are
			 * available
			 */
			netif_stop_queue(netdev);
			status = NETDEV_TX_BUSY;
		} else {
			status = NETDEV_TX_OK;
		}
	}
	return status;
}

void et131x_tx_timeout(struct net_device *netdev)
{
	struct et131x_adapter *etdev = netdev_priv(netdev);
	struct tcb *tcb;
	unsigned long flags;

	/* Just skip this part if the adapter is doing link detection */
	if (etdev->Flags & fMP_ADAPTER_LINK_DETECTION)
		return;

	/* Any nonrecoverable hardware error?
	 * Checks adapter->flags for any failure in phy reading
	 */
	if (etdev->Flags & fMP_ADAPTER_NON_RECOVER_ERROR)
		return;

	/* Hardware failure? */
	if (etdev->Flags & fMP_ADAPTER_HARDWARE_ERROR) {
		dev_err(&etdev->pdev->dev, "hardware error - reset\n");
		return;
	}

	/* Is send stuck? */
	spin_lock_irqsave(&etdev->TCBSendQLock, flags);

	tcb = etdev->tx_ring.send_head;

	if (tcb != NULL) {
		tcb->count++;

		if (tcb->count > NIC_SEND_HANG_THRESHOLD) {
			spin_unlock_irqrestore(&etdev->TCBSendQLock,
					       flags);

			dev_warn(&etdev->pdev->dev,
				"Send stuck - reset.  tcb->WrIndex %x, Flags 0x%08x\n",
				tcb->index,
				tcb->flags);

			et131x_close(netdev);
			et131x_open(netdev);

			return;
		}
	}

	spin_unlock_irqrestore(&etdev->TCBSendQLock, flags);
}

int et131x_change_mtu(struct net_device *netdev, int new_mtu)
{
	int result = 0;
	struct et131x_adapter *adapter = netdev_priv(netdev);

	/* Make sure the requested MTU is valid */
	if (new_mtu < 64 || new_mtu > 9216)
		return -EINVAL;

	/* Stop the netif queue */
	netif_stop_queue(netdev);

	/* Stop the Tx and Rx DMA engines */
	et131x_rx_dma_disable(adapter);
	et131x_tx_dma_disable(adapter);

	/* Disable device interrupts */
	et131x_disable_interrupts(adapter);
	et131x_handle_send_interrupt(adapter);
	et131x_handle_recv_interrupt(adapter);

	/* Set the new MTU */
	netdev->mtu = new_mtu;

	/* Free Rx DMA memory */
	et131x_adapter_memory_free(adapter);

	/* Set the config parameter for Jumbo Packet support */
	adapter->RegistryJumboPacket = new_mtu + 14;
	et131x_soft_reset(adapter);

	/* Alloc and init Rx DMA memory */
	result = et131x_adapter_memory_alloc(adapter);
	if (result != 0) {
		dev_warn(&adapter->pdev->dev,
			"Change MTU failed; couldn't re-alloc DMA memory\n");
		return result;
	}

	et131x_init_send(adapter);

	et131x_hwaddr_init(adapter);
	memcpy(netdev->dev_addr, adapter->CurrentAddress, ETH_ALEN);

	/* Init the device with the new settings */
	et131x_adapter_setup(adapter);

	/* Enable interrupts */
	if (adapter->Flags & fMP_ADAPTER_INTERRUPT_IN_USE)
		et131x_enable_interrupts(adapter);

	/* Restart the Tx and Rx DMA engines */
	et131x_rx_dma_enable(adapter);
	et131x_tx_dma_enable(adapter);

	/* Restart the netif queue */
	netif_wake_queue(netdev);
	return result;
}

int et131x_set_mac_addr(struct net_device *netdev, void *new_mac)
{
	int result = 0;
	struct et131x_adapter *adapter = netdev_priv(netdev);
	struct sockaddr *address = new_mac;

	/* begin blux */

	if (adapter == NULL)
		return -ENODEV;

	/* Make sure the requested MAC is valid */
	if (!is_valid_ether_addr(address->sa_data))
		return -EINVAL;

	/* Stop the netif queue */
	netif_stop_queue(netdev);

	/* Stop the Tx and Rx DMA engines */
	et131x_rx_dma_disable(adapter);
	et131x_tx_dma_disable(adapter);

	/* Disable device interrupts */
	et131x_disable_interrupts(adapter);
	et131x_handle_send_interrupt(adapter);
	et131x_handle_recv_interrupt(adapter);

	/* Set the new MAC */
	/* netdev->set_mac_address  = &new_mac; */
	/* netdev->mtu = new_mtu; */

	memcpy(netdev->dev_addr, address->sa_data, netdev->addr_len);

	printk(KERN_INFO "%s: Setting MAC address to %pM\n",
			netdev->name, netdev->dev_addr);

	/* Free Rx DMA memory */
	et131x_adapter_memory_free(adapter);

	/* Set the config parameter for Jumbo Packet support */
	/* adapter->RegistryJumboPacket = new_mtu + 14; */
	/* blux: not needet here, we'll change the MAC */

	et131x_soft_reset(adapter);

	/* Alloc and init Rx DMA memory */
	result = et131x_adapter_memory_alloc(adapter);
	if (result != 0) {
		dev_err(&adapter->pdev->dev,
			"Change MAC failed; couldn't re-alloc DMA memory\n");
		return result;
	}

	et131x_init_send(adapter);

	et131x_hwaddr_init(adapter);

	/* Init the device with the new settings */
	et131x_adapter_setup(adapter);

	/* Enable interrupts */
	if (adapter->Flags & fMP_ADAPTER_INTERRUPT_IN_USE)
		et131x_enable_interrupts(adapter);

	/* Restart the Tx and Rx DMA engines */
	et131x_rx_dma_enable(adapter);
	et131x_tx_dma_enable(adapter);

	/* Restart the netif queue */
	netif_wake_queue(netdev);
	return result;
}
