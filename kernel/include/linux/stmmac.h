

#ifndef __STMMAC_PLATFORM_DATA
#define __STMMAC_PLATFORM_DATA

/* platfrom data for platfrom device structure's platfrom_data field */

/* Private data for the STM on-board ethernet driver */
struct plat_stmmacenet_data {
	int bus_id;
	int pbl;
	int has_gmac;
	int enh_desc;
	void (*fix_mac_speed)(void *priv, unsigned int speed);
	void (*bus_setup)(unsigned long ioaddr);
#ifdef CONFIG_STM_DRIVERS
	struct stm_pad_config *pad_config;
#endif
	void *bsp_priv;
};

struct plat_stmmacphy_data {
	int bus_id;
	int phy_addr;
	unsigned int phy_mask;
	int interface;
	int (*phy_reset)(void *priv);
	void *priv;
};
#endif

