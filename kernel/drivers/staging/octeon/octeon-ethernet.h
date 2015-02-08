

#ifndef OCTEON_ETHERNET_H
#define OCTEON_ETHERNET_H

struct octeon_ethernet {
	/* PKO hardware output port */
	int port;
	/* PKO hardware queue for the port */
	int queue;
	/* Hardware fetch and add to count outstanding tx buffers */
	int fau;
	/*
	 * Type of port. This is one of the enums in
	 * cvmx_helper_interface_mode_t
	 */
	int imode;
	/* List of outstanding tx buffers per queue */
	struct sk_buff_head tx_free_list[16];
	/* Device statistics */
	struct net_device_stats stats;
	struct phy_device *phydev;
	unsigned int last_link;
	/* Last negotiated link state */
	uint64_t link_info;
	/* Called periodically to check link status */
	void (*poll) (struct net_device *dev);
	struct delayed_work	port_periodic_work;
	struct work_struct	port_work;	/* may be unused. */
};

int cvm_oct_free_work(void *work_queue_entry);

extern int cvm_oct_rgmii_init(struct net_device *dev);
extern void cvm_oct_rgmii_uninit(struct net_device *dev);
extern int cvm_oct_rgmii_open(struct net_device *dev);
extern int cvm_oct_rgmii_stop(struct net_device *dev);

extern int cvm_oct_sgmii_init(struct net_device *dev);
extern void cvm_oct_sgmii_uninit(struct net_device *dev);
extern int cvm_oct_sgmii_open(struct net_device *dev);
extern int cvm_oct_sgmii_stop(struct net_device *dev);

extern int cvm_oct_spi_init(struct net_device *dev);
extern void cvm_oct_spi_uninit(struct net_device *dev);
extern int cvm_oct_xaui_init(struct net_device *dev);
extern void cvm_oct_xaui_uninit(struct net_device *dev);
extern int cvm_oct_xaui_open(struct net_device *dev);
extern int cvm_oct_xaui_stop(struct net_device *dev);

extern int cvm_oct_common_init(struct net_device *dev);
extern void cvm_oct_common_uninit(struct net_device *dev);

extern int always_use_pow;
extern int pow_send_group;
extern int pow_receive_group;
extern char pow_send_list[];
extern struct net_device *cvm_oct_device[];
extern struct workqueue_struct *cvm_oct_poll_queue;
extern atomic_t cvm_oct_poll_queue_stopping;
extern u64 cvm_oct_tx_poll_interval;

extern int max_rx_cpus;
extern int rx_napi_weight;

#endif
