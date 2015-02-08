

#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/slab.h>

#include "stmmac.h"

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

static int stmmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	struct net_device *ndev = bus->priv;
	struct stmmac_priv *priv = netdev_priv(ndev);
	unsigned long ioaddr = ndev->base_addr;
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;

	int data;
	u16 regValue = (((phyaddr << 11) & (0x0000F800)) |
			((phyreg << 6) & (0x000007C0)));
	regValue |= MII_BUSY;	/* in case of GMAC */

	do {} while (((readl(ioaddr + mii_address)) & MII_BUSY) == 1);
	writel(regValue, ioaddr + mii_address);
	do {} while (((readl(ioaddr + mii_address)) & MII_BUSY) == 1);

	/* Read the data from the MII data register */
	data = (int)readl(ioaddr + mii_data);

	return data;
}

static int stmmac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	struct net_device *ndev = bus->priv;
	struct stmmac_priv *priv = netdev_priv(ndev);
	unsigned long ioaddr = ndev->base_addr;
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;

	u16 value =
	    (((phyaddr << 11) & (0x0000F800)) | ((phyreg << 6) & (0x000007C0)))
	    | MII_WRITE;

	value |= MII_BUSY;

	/* Wait until any existing MII operation is complete */
	do {} while (((readl(ioaddr + mii_address)) & MII_BUSY) == 1);

	/* Set the MII address register to write */
	writel(phydata, ioaddr + mii_data);
	writel(value, ioaddr + mii_address);

	/* Wait until any existing MII operation is complete */
	do {} while (((readl(ioaddr + mii_address)) & MII_BUSY) == 1);

	return 0;
}

static int stmmac_mdio_reset(struct mii_bus *bus)
{
	struct net_device *ndev = bus->priv;
	struct stmmac_priv *priv = netdev_priv(ndev);
	unsigned long ioaddr = ndev->base_addr;
	unsigned int mii_address = priv->hw->mii.addr;

	if (priv->phy_reset) {
		pr_debug("stmmac_mdio_reset: calling phy_reset\n");
		priv->phy_reset(priv->bsp_priv);
	}

	/* This is a workaround for problems with the STE101P PHY.
	 * It doesn't complete its reset until at least one clock cycle
	 * on MDC, so perform a dummy mdio read.
	 */
	writel(0, ioaddr + mii_address);

	return 0;
}

int stmmac_mdio_register(struct net_device *ndev)
{
	int err = 0;
	struct mii_bus *new_bus;
	int *irqlist;
	struct stmmac_priv *priv = netdev_priv(ndev);
	int addr, found;

	new_bus = mdiobus_alloc();
	if (new_bus == NULL)
		return -ENOMEM;

	irqlist = kzalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (irqlist == NULL) {
		err = -ENOMEM;
		goto irqlist_alloc_fail;
	}

	/* Assign IRQ to phy at address phy_addr */
	if (priv->phy_addr != -1)
		irqlist[priv->phy_addr] = priv->phy_irq;

	new_bus->name = "STMMAC MII Bus";
	new_bus->read = &stmmac_mdio_read;
	new_bus->write = &stmmac_mdio_write;
	new_bus->reset = &stmmac_mdio_reset;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%x", priv->bus_id);
	new_bus->priv = ndev;
	new_bus->irq = irqlist;
	new_bus->phy_mask = priv->phy_mask;
	new_bus->parent = priv->device;
	err = mdiobus_register(new_bus);
	if (err != 0) {
		pr_err("%s: Cannot register as MDIO bus\n", new_bus->name);
		goto bus_register_fail;
	}

	priv->mii = new_bus;

	found = 0;
	for (addr = 0; addr < 32; addr++) {
		struct phy_device *phydev = new_bus->phy_map[addr];
		if (phydev) {
			if (priv->phy_addr == -1) {
				priv->phy_addr = addr;
				phydev->irq = priv->phy_irq;
				irqlist[addr] = priv->phy_irq;
			}
			pr_info("%s: PHY ID %08x at %d IRQ %d (%s)%s\n",
			       ndev->name, phydev->phy_id, addr,
			       phydev->irq, dev_name(&phydev->dev),
			       (addr == priv->phy_addr) ? " active" : "");
			found = 1;
		}
	}

	if (!found)
		pr_warning("%s: No PHY found\n", ndev->name);

	return 0;
bus_register_fail:
	kfree(irqlist);
irqlist_alloc_fail:
	kfree(new_bus);
	return err;
}

int stmmac_mdio_unregister(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	kfree(priv->mii);

	return 0;
}
