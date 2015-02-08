

#ifndef __PHY_H
#define __PHY_H

#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/mod_devicetable.h>

#include <asm/atomic.h>

#define PHY_BASIC_FEATURES	(SUPPORTED_10baseT_Half | \
				 SUPPORTED_10baseT_Full | \
				 SUPPORTED_100baseT_Half | \
				 SUPPORTED_100baseT_Full | \
				 SUPPORTED_Autoneg | \
				 SUPPORTED_TP | \
				 SUPPORTED_MII)

#define PHY_GBIT_FEATURES	(PHY_BASIC_FEATURES | \
				 SUPPORTED_1000baseT_Half | \
				 SUPPORTED_1000baseT_Full)

#define PHY_POLL		-1
#define PHY_IGNORE_INTERRUPT	-2

#define PHY_HAS_INTERRUPT	0x00000001
#define PHY_HAS_MAGICANEG	0x00000002

/* Interface Mode definitions */
typedef enum {
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI
} phy_interface_t;


#define PHY_INIT_TIMEOUT	100000
#define PHY_STATE_TIME		1
#define PHY_FORCE_TIMEOUT	10
#define PHY_AN_TIMEOUT		10

#define PHY_MAX_ADDR	32

/* Used when trying to connect to a specific phy (mii bus id:phy device id) */
#define PHY_ID_FMT "%s:%02x"

#define MII_BUS_ID_SIZE	(20 - 3)

#define MII_ADDR_C45 (1<<30)

struct mii_bus {
	const char *name;
	char id[MII_BUS_ID_SIZE];
	void *priv;
	int (*read)(struct mii_bus *bus, int phy_id, int regnum);
	int (*write)(struct mii_bus *bus, int phy_id, int regnum, u16 val);
	int (*reset)(struct mii_bus *bus);

	/*
	 * A lock to ensure that only one thing can read/write
	 * the MDIO bus at a time
	 */
	struct mutex mdio_lock;

	struct device *parent;
	enum {
		MDIOBUS_ALLOCATED = 1,
		MDIOBUS_REGISTERED,
		MDIOBUS_UNREGISTERED,
		MDIOBUS_RELEASED,
	} state;
	struct device dev;

	/* list of all PHYs on bus */
	struct phy_device *phy_map[PHY_MAX_ADDR];

	/* Phy addresses to be ignored when probing */
	u32 phy_mask;

	/*
	 * Pointer to an array of interrupts, each PHY's
	 * interrupt at the index matching its address
	 */
	int *irq;
};
#define to_mii_bus(d) container_of(d, struct mii_bus, dev)

struct mii_bus *mdiobus_alloc(void);
int mdiobus_register(struct mii_bus *bus);
void mdiobus_unregister(struct mii_bus *bus);
void mdiobus_free(struct mii_bus *bus);
struct phy_device *mdiobus_scan(struct mii_bus *bus, int addr);
int mdiobus_read(struct mii_bus *bus, int addr, u32 regnum);
int mdiobus_write(struct mii_bus *bus, int addr, u32 regnum, u16 val);


#define PHY_INTERRUPT_DISABLED	0x0
#define PHY_INTERRUPT_ENABLED	0x80000000

enum phy_state {
	PHY_DOWN=0,
	PHY_STARTING,
	PHY_READY,
	PHY_PENDING,
	PHY_UP,
	PHY_AN,
	PHY_RUNNING,
	PHY_NOLINK,
	PHY_FORCING,
	PHY_CHANGELINK,
	PHY_HALTED,
	PHY_RESUMING
};

struct phy_device {
	/* Information about the PHY type */
	/* And management functions */
	struct phy_driver *drv;

	struct mii_bus *bus;

	struct device dev;

	u32 phy_id;

	enum phy_state state;

	u32 dev_flags;

	phy_interface_t interface;

	/* Bus address of the PHY (0-32) */
	int addr;

	/*
	 * forced speed & duplex (no autoneg)
	 * partner speed & duplex & pause (autoneg)
	 */
	int speed;
	int duplex;
	int pause;
	int asym_pause;

	/* The most recently read link state */
	int link;

	/* Enabled Interrupts */
	u32 interrupts;

	/* Union of PHY and Attached devices' supported modes */
	/* See mii.h for more info */
	u32 supported;
	u32 advertising;

	int autoneg;

	int link_timeout;

	/*
	 * Interrupt number for this PHY
	 * -1 means no interrupt
	 */
	int irq;

	/* private data pointer */
	/* For use by PHYs to maintain extra state */
	void *priv;

	/* Interrupt and Polling infrastructure */
	struct work_struct phy_queue;
	struct delayed_work state_queue;
	atomic_t irq_disable;

	struct mutex lock;

	struct net_device *attached_dev;

	void (*adjust_link)(struct net_device *dev);

	void (*adjust_state)(struct net_device *dev);
};
#define to_phy_device(d) container_of(d, struct phy_device, dev)

struct phy_driver {
	u32 phy_id;
	char *name;
	unsigned int phy_id_mask;
	u32 features;
	u32 flags;

	/*
	 * Called to initialize the PHY,
	 * including after a reset
	 */
	int (*config_init)(struct phy_device *phydev);

	/*
	 * Called during discovery.  Used to set
	 * up device-specific structures, if any
	 */
	int (*probe)(struct phy_device *phydev);

	/* PHY Power Management */
	int (*suspend)(struct phy_device *phydev);
	int (*resume)(struct phy_device *phydev);

	/*
	 * Configures the advertisement and resets
	 * autonegotiation if phydev->autoneg is on,
	 * forces the speed to the current settings in phydev
	 * if phydev->autoneg is off
	 */
	int (*config_aneg)(struct phy_device *phydev);

	/* Determines the negotiated speed and duplex */
	int (*read_status)(struct phy_device *phydev);

	/* Clears any pending interrupts */
	int (*ack_interrupt)(struct phy_device *phydev);

	/* Enables or disables interrupts */
	int (*config_intr)(struct phy_device *phydev);

	/*
	 * Checks if the PHY generated an interrupt.
	 * For multi-PHY devices with shared PHY interrupt pin
	 */
	int (*did_interrupt)(struct phy_device *phydev);

	/* Clears up any memory if needed */
	void (*remove)(struct phy_device *phydev);

	struct device_driver driver;
};
#define to_phy_driver(d) container_of(d, struct phy_driver, driver)

#define PHY_ANY_ID "MATCH ANY PHY"
#define PHY_ANY_UID 0xffffffff

/* A Structure for boards to register fixups with the PHY Lib */
struct phy_fixup {
	struct list_head list;
	char bus_id[20];
	u32 phy_uid;
	u32 phy_uid_mask;
	int (*run)(struct phy_device *phydev);
};

static inline int phy_read(struct phy_device *phydev, u32 regnum)
{
	return mdiobus_read(phydev->bus, phydev->addr, regnum);
}

static inline int phy_write(struct phy_device *phydev, u32 regnum, u16 val)
{
	return mdiobus_write(phydev->bus, phydev->addr, regnum, val);
}

int get_phy_id(struct mii_bus *bus, int addr, u32 *phy_id);
struct phy_device* get_phy_device(struct mii_bus *bus, int addr);
int phy_device_register(struct phy_device *phy);
int phy_clear_interrupt(struct phy_device *phydev);
int phy_config_interrupt(struct phy_device *phydev, u32 interrupts);
int phy_init_hw(struct phy_device *phydev);
int phy_attach_direct(struct net_device *dev, struct phy_device *phydev,
		u32 flags, phy_interface_t interface);
struct phy_device * phy_attach(struct net_device *dev,
		const char *bus_id, u32 flags, phy_interface_t interface);
struct phy_device *phy_find_first(struct mii_bus *bus);
int phy_connect_direct(struct net_device *dev, struct phy_device *phydev,
		void (*handler)(struct net_device *), u32 flags,
		phy_interface_t interface);
struct phy_device * phy_connect(struct net_device *dev, const char *bus_id,
		void (*handler)(struct net_device *), u32 flags,
		phy_interface_t interface);
void phy_disconnect(struct phy_device *phydev);
void phy_detach(struct phy_device *phydev);
void phy_start(struct phy_device *phydev);
void phy_stop(struct phy_device *phydev);
int phy_start_aneg(struct phy_device *phydev);

void phy_sanitize_settings(struct phy_device *phydev);
int phy_stop_interrupts(struct phy_device *phydev);
int phy_enable_interrupts(struct phy_device *phydev);
int phy_disable_interrupts(struct phy_device *phydev);

static inline int phy_read_status(struct phy_device *phydev) {
	return phydev->drv->read_status(phydev);
}

int genphy_config_advert(struct phy_device *phydev);
int genphy_setup_forced(struct phy_device *phydev);
int genphy_restart_aneg(struct phy_device *phydev);
int genphy_config_aneg(struct phy_device *phydev);
int genphy_update_link(struct phy_device *phydev);
int genphy_read_status(struct phy_device *phydev);
int genphy_suspend(struct phy_device *phydev);
int genphy_resume(struct phy_device *phydev);
void phy_driver_unregister(struct phy_driver *drv);
int phy_driver_register(struct phy_driver *new_driver);
void phy_prepare_link(struct phy_device *phydev,
		void (*adjust_link)(struct net_device *));
void phy_state_machine(struct work_struct *work);
void phy_start_machine(struct phy_device *phydev,
		void (*handler)(struct net_device *));
void phy_stop_machine(struct phy_device *phydev);
int phy_ethtool_sset(struct phy_device *phydev, struct ethtool_cmd *cmd);
int phy_ethtool_gset(struct phy_device *phydev, struct ethtool_cmd *cmd);
int phy_mii_ioctl(struct phy_device *phydev,
		struct mii_ioctl_data *mii_data, int cmd);
int phy_start_interrupts(struct phy_device *phydev);
void phy_print_status(struct phy_device *phydev);
struct phy_device* phy_device_create(struct mii_bus *bus, int addr, int phy_id);
void phy_device_free(struct phy_device *phydev);

int phy_register_fixup(const char *bus_id, u32 phy_uid, u32 phy_uid_mask,
		int (*run)(struct phy_device *));
int phy_register_fixup_for_id(const char *bus_id,
		int (*run)(struct phy_device *));
int phy_register_fixup_for_uid(u32 phy_uid, u32 phy_uid_mask,
		int (*run)(struct phy_device *));
int phy_scan_fixups(struct phy_device *phydev);

int __init mdio_bus_init(void);
void mdio_bus_exit(void);

extern struct bus_type mdio_bus_type;
#endif /* __PHY_H */
