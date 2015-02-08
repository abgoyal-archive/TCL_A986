
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

void phy_print_status(struct phy_device *phydev)
{
	pr_info("PHY: %s - Link is %s", dev_name(&phydev->dev),
			phydev->link ? "Up" : "Down");
	if (phydev->link)
		printk(" - %d/%s", phydev->speed,
				DUPLEX_FULL == phydev->duplex ?
				"Full" : "Half");

	printk("\n");
}
EXPORT_SYMBOL(phy_print_status);


int phy_clear_interrupt(struct phy_device *phydev)
{
	int err = 0;

	if (phydev->drv->ack_interrupt)
		err = phydev->drv->ack_interrupt(phydev);

	return err;
}

int phy_config_interrupt(struct phy_device *phydev, u32 interrupts)
{
	int err = 0;

	phydev->interrupts = interrupts;
	if (phydev->drv->config_intr)
		err = phydev->drv->config_intr(phydev);

	return err;
}


static inline int phy_aneg_done(struct phy_device *phydev)
{
	int retval;

	retval = phy_read(phydev, MII_BMSR);

	return (retval < 0) ? retval : (retval & BMSR_ANEGCOMPLETE);
}

struct phy_setting {
	int speed;
	int duplex;
	u32 setting;
};

/* A mapping of all SUPPORTED settings to speed/duplex */
static const struct phy_setting settings[] = {
	{
		.speed = 10000,
		.duplex = DUPLEX_FULL,
		.setting = SUPPORTED_10000baseT_Full,
	},
	{
		.speed = SPEED_1000,
		.duplex = DUPLEX_FULL,
		.setting = SUPPORTED_1000baseT_Full,
	},
	{
		.speed = SPEED_1000,
		.duplex = DUPLEX_HALF,
		.setting = SUPPORTED_1000baseT_Half,
	},
	{
		.speed = SPEED_100,
		.duplex = DUPLEX_FULL,
		.setting = SUPPORTED_100baseT_Full,
	},
	{
		.speed = SPEED_100,
		.duplex = DUPLEX_HALF,
		.setting = SUPPORTED_100baseT_Half,
	},
	{
		.speed = SPEED_10,
		.duplex = DUPLEX_FULL,
		.setting = SUPPORTED_10baseT_Full,
	},
	{
		.speed = SPEED_10,
		.duplex = DUPLEX_HALF,
		.setting = SUPPORTED_10baseT_Half,
	},
};

#define MAX_NUM_SETTINGS ARRAY_SIZE(settings)

static inline int phy_find_setting(int speed, int duplex)
{
	int idx = 0;

	while (idx < ARRAY_SIZE(settings) &&
			(settings[idx].speed != speed ||
			settings[idx].duplex != duplex))
		idx++;

	return idx < MAX_NUM_SETTINGS ? idx : MAX_NUM_SETTINGS - 1;
}

static inline int phy_find_valid(int idx, u32 features)
{
	while (idx < MAX_NUM_SETTINGS && !(settings[idx].setting & features))
		idx++;

	return idx < MAX_NUM_SETTINGS ? idx : MAX_NUM_SETTINGS - 1;
}

void phy_sanitize_settings(struct phy_device *phydev)
{
	u32 features = phydev->supported;
	int idx;

	/* Sanitize settings based on PHY capabilities */
	if ((features & SUPPORTED_Autoneg) == 0)
		phydev->autoneg = AUTONEG_DISABLE;

	idx = phy_find_valid(phy_find_setting(phydev->speed, phydev->duplex),
			features);

	phydev->speed = settings[idx].speed;
	phydev->duplex = settings[idx].duplex;
}
EXPORT_SYMBOL(phy_sanitize_settings);

int phy_ethtool_sset(struct phy_device *phydev, struct ethtool_cmd *cmd)
{
	if (cmd->phy_address != phydev->addr)
		return -EINVAL;

	/* We make sure that we don't pass unsupported
	 * values in to the PHY */
	cmd->advertising &= phydev->supported;

	/* Verify the settings we care about. */
	if (cmd->autoneg != AUTONEG_ENABLE && cmd->autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_ENABLE && cmd->advertising == 0)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_DISABLE &&
	    ((cmd->speed != SPEED_1000 &&
	      cmd->speed != SPEED_100 &&
	      cmd->speed != SPEED_10) ||
	     (cmd->duplex != DUPLEX_HALF &&
	      cmd->duplex != DUPLEX_FULL)))
		return -EINVAL;

	phydev->autoneg = cmd->autoneg;

	phydev->speed = cmd->speed;

	phydev->advertising = cmd->advertising;

	if (AUTONEG_ENABLE == cmd->autoneg)
		phydev->advertising |= ADVERTISED_Autoneg;
	else
		phydev->advertising &= ~ADVERTISED_Autoneg;

	phydev->duplex = cmd->duplex;

	/* Restart the PHY */
	phy_start_aneg(phydev);

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_sset);

int phy_ethtool_gset(struct phy_device *phydev, struct ethtool_cmd *cmd)
{
	cmd->supported = phydev->supported;

	cmd->advertising = phydev->advertising;

	cmd->speed = phydev->speed;
	cmd->duplex = phydev->duplex;
	cmd->port = PORT_MII;
	cmd->phy_address = phydev->addr;
	cmd->transceiver = XCVR_EXTERNAL;
	cmd->autoneg = phydev->autoneg;

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_gset);

int phy_mii_ioctl(struct phy_device *phydev,
		struct mii_ioctl_data *mii_data, int cmd)
{
	u16 val = mii_data->val_in;

	switch (cmd) {
	case SIOCGMIIPHY:
		mii_data->phy_id = phydev->addr;
		/* fall through */

	case SIOCGMIIREG:
		mii_data->val_out = phy_read(phydev, mii_data->reg_num);
		break;

	case SIOCSMIIREG:
		if (mii_data->phy_id == phydev->addr) {
			switch(mii_data->reg_num) {
			case MII_BMCR:
				if ((val & (BMCR_RESET|BMCR_ANENABLE)) == 0)
					phydev->autoneg = AUTONEG_DISABLE;
				else
					phydev->autoneg = AUTONEG_ENABLE;
				if ((!phydev->autoneg) && (val & BMCR_FULLDPLX))
					phydev->duplex = DUPLEX_FULL;
				else
					phydev->duplex = DUPLEX_HALF;
				if ((!phydev->autoneg) &&
						(val & BMCR_SPEED1000))
					phydev->speed = SPEED_1000;
				else if ((!phydev->autoneg) &&
						(val & BMCR_SPEED100))
					phydev->speed = SPEED_100;
				break;
			case MII_ADVERTISE:
				phydev->advertising = val;
				break;
			default:
				/* do nothing */
				break;
			}
		}

		phy_write(phydev, mii_data->reg_num, val);
		
		if (mii_data->reg_num == MII_BMCR &&
		    val & BMCR_RESET &&
		    phydev->drv->config_init) {
			phy_scan_fixups(phydev);
			phydev->drv->config_init(phydev);
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}
EXPORT_SYMBOL(phy_mii_ioctl);

int phy_start_aneg(struct phy_device *phydev)
{
	int err;

	mutex_lock(&phydev->lock);

	if (AUTONEG_DISABLE == phydev->autoneg)
		phy_sanitize_settings(phydev);

	err = phydev->drv->config_aneg(phydev);

	if (err < 0)
		goto out_unlock;

	if (phydev->state != PHY_HALTED) {
		if (AUTONEG_ENABLE == phydev->autoneg) {
			phydev->state = PHY_AN;
			phydev->link_timeout = PHY_AN_TIMEOUT;
		} else {
			phydev->state = PHY_FORCING;
			phydev->link_timeout = PHY_FORCE_TIMEOUT;
		}
	}

out_unlock:
	mutex_unlock(&phydev->lock);
	return err;
}
EXPORT_SYMBOL(phy_start_aneg);


static void phy_change(struct work_struct *work);

void phy_start_machine(struct phy_device *phydev,
		void (*handler)(struct net_device *))
{
	phydev->adjust_state = handler;

	schedule_delayed_work(&phydev->state_queue, HZ);
}

void phy_stop_machine(struct phy_device *phydev)
{
	cancel_delayed_work_sync(&phydev->state_queue);

	mutex_lock(&phydev->lock);
	if (phydev->state > PHY_UP)
		phydev->state = PHY_UP;
	mutex_unlock(&phydev->lock);

	phydev->adjust_state = NULL;
}

static void phy_force_reduction(struct phy_device *phydev)
{
	int idx;

	idx = phy_find_setting(phydev->speed, phydev->duplex);
	
	idx++;

	idx = phy_find_valid(idx, phydev->supported);

	phydev->speed = settings[idx].speed;
	phydev->duplex = settings[idx].duplex;

	pr_info("Trying %d/%s\n", phydev->speed,
			DUPLEX_FULL == phydev->duplex ?
			"FULL" : "HALF");
}


static void phy_error(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);
	phydev->state = PHY_HALTED;
	mutex_unlock(&phydev->lock);
}

static irqreturn_t phy_interrupt(int irq, void *phy_dat)
{
	struct phy_device *phydev = phy_dat;

	if (PHY_HALTED == phydev->state)
		return IRQ_NONE;		/* It can't be ours.  */

	/* The MDIO bus is not allowed to be written in interrupt
	 * context, so we need to disable the irq here.  A work
	 * queue will write the PHY to disable and clear the
	 * interrupt, and then reenable the irq line. */
	disable_irq_nosync(irq);
	atomic_inc(&phydev->irq_disable);

	schedule_work(&phydev->phy_queue);

	return IRQ_HANDLED;
}

int phy_enable_interrupts(struct phy_device *phydev)
{
	int err;

	err = phy_clear_interrupt(phydev);

	if (err < 0)
		return err;

	err = phy_config_interrupt(phydev, PHY_INTERRUPT_ENABLED);

	return err;
}
EXPORT_SYMBOL(phy_enable_interrupts);

int phy_disable_interrupts(struct phy_device *phydev)
{
	int err;

	/* Disable PHY interrupts */
	err = phy_config_interrupt(phydev, PHY_INTERRUPT_DISABLED);

	if (err)
		goto phy_err;

	/* Clear the interrupt */
	err = phy_clear_interrupt(phydev);

	if (err)
		goto phy_err;

	return 0;

phy_err:
	phy_error(phydev);

	return err;
}
EXPORT_SYMBOL(phy_disable_interrupts);

int phy_start_interrupts(struct phy_device *phydev)
{
	int err = 0;

	INIT_WORK(&phydev->phy_queue, phy_change);

	atomic_set(&phydev->irq_disable, 0);
	if (request_irq(phydev->irq, phy_interrupt,
				IRQF_SHARED,
				"phy_interrupt",
				phydev) < 0) {
		printk(KERN_WARNING "%s: Can't get IRQ %d (PHY)\n",
				phydev->bus->name,
				phydev->irq);
		phydev->irq = PHY_POLL;
		return 0;
	}

	err = phy_enable_interrupts(phydev);

	return err;
}
EXPORT_SYMBOL(phy_start_interrupts);

int phy_stop_interrupts(struct phy_device *phydev)
{
	int err;

	err = phy_disable_interrupts(phydev);

	if (err)
		phy_error(phydev);

	free_irq(phydev->irq, phydev);

	/*
	 * Cannot call flush_scheduled_work() here as desired because
	 * of rtnl_lock(), but we do not really care about what would
	 * be done, except from enable_irq(), so cancel any work
	 * possibly pending and take care of the matter below.
	 */
	cancel_work_sync(&phydev->phy_queue);
	/*
	 * If work indeed has been cancelled, disable_irq() will have
	 * been left unbalanced from phy_interrupt() and enable_irq()
	 * has to be called so that other devices on the line work.
	 */
	while (atomic_dec_return(&phydev->irq_disable) >= 0)
		enable_irq(phydev->irq);

	return err;
}
EXPORT_SYMBOL(phy_stop_interrupts);


static void phy_change(struct work_struct *work)
{
	int err;
	struct phy_device *phydev =
		container_of(work, struct phy_device, phy_queue);

	if (phydev->drv->did_interrupt &&
	    !phydev->drv->did_interrupt(phydev))
		goto ignore;

	err = phy_disable_interrupts(phydev);

	if (err)
		goto phy_err;

	mutex_lock(&phydev->lock);
	if ((PHY_RUNNING == phydev->state) || (PHY_NOLINK == phydev->state))
		phydev->state = PHY_CHANGELINK;
	mutex_unlock(&phydev->lock);

	atomic_dec(&phydev->irq_disable);
	enable_irq(phydev->irq);

	/* Reenable interrupts */
	if (PHY_HALTED != phydev->state)
		err = phy_config_interrupt(phydev, PHY_INTERRUPT_ENABLED);

	if (err)
		goto irq_enable_err;

	/* reschedule state queue work to run as soon as possible */
	cancel_delayed_work_sync(&phydev->state_queue);
	schedule_delayed_work(&phydev->state_queue, 0);

	return;

ignore:
	atomic_dec(&phydev->irq_disable);
	enable_irq(phydev->irq);
	return;

irq_enable_err:
	disable_irq(phydev->irq);
	atomic_inc(&phydev->irq_disable);
phy_err:
	phy_error(phydev);
}

void phy_stop(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);

	if (PHY_HALTED == phydev->state)
		goto out_unlock;

	if (phydev->irq != PHY_POLL) {
		/* Disable PHY Interrupts */
		phy_config_interrupt(phydev, PHY_INTERRUPT_DISABLED);

		/* Clear any pending interrupts */
		phy_clear_interrupt(phydev);
	}

	phydev->state = PHY_HALTED;

out_unlock:
	mutex_unlock(&phydev->lock);

	/*
	 * Cannot call flush_scheduled_work() here as desired because
	 * of rtnl_lock(), but PHY_HALTED shall guarantee phy_change()
	 * will not reenable interrupts.
	 */
}


void phy_start(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);

	switch (phydev->state) {
		case PHY_STARTING:
			phydev->state = PHY_PENDING;
			break;
		case PHY_READY:
			phydev->state = PHY_UP;
			break;
		case PHY_HALTED:
			phydev->state = PHY_RESUMING;
		default:
			break;
	}
	mutex_unlock(&phydev->lock);
}
EXPORT_SYMBOL(phy_stop);
EXPORT_SYMBOL(phy_start);

void phy_state_machine(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct phy_device *phydev =
			container_of(dwork, struct phy_device, state_queue);
	int needs_aneg = 0;
	int err = 0;

	mutex_lock(&phydev->lock);

	if (phydev->adjust_state)
		phydev->adjust_state(phydev->attached_dev);

	switch(phydev->state) {
		case PHY_DOWN:
		case PHY_STARTING:
		case PHY_READY:
		case PHY_PENDING:
			break;
		case PHY_UP:
			needs_aneg = 1;

			phydev->link_timeout = PHY_AN_TIMEOUT;

			break;
		case PHY_AN:
			err = phy_read_status(phydev);

			if (err < 0)
				break;

			/* If the link is down, give up on
			 * negotiation for now */
			if (!phydev->link) {
				phydev->state = PHY_NOLINK;
				netif_carrier_off(phydev->attached_dev);
				phydev->adjust_link(phydev->attached_dev);
				break;
			}

			/* Check if negotiation is done.  Break
			 * if there's an error */
			err = phy_aneg_done(phydev);
			if (err < 0)
				break;

			/* If AN is done, we're running */
			if (err > 0) {
				phydev->state = PHY_RUNNING;
				netif_carrier_on(phydev->attached_dev);
				phydev->adjust_link(phydev->attached_dev);

			} else if (0 == phydev->link_timeout--) {
				int idx;

				needs_aneg = 1;
				/* If we have the magic_aneg bit,
				 * we try again */
				if (phydev->drv->flags & PHY_HAS_MAGICANEG)
					break;

				/* The timer expired, and we still
				 * don't have a setting, so we try
				 * forcing it until we find one that
				 * works, starting from the fastest speed,
				 * and working our way down */
				idx = phy_find_valid(0, phydev->supported);

				phydev->speed = settings[idx].speed;
				phydev->duplex = settings[idx].duplex;

				phydev->autoneg = AUTONEG_DISABLE;

				pr_info("Trying %d/%s\n", phydev->speed,
						DUPLEX_FULL ==
						phydev->duplex ?
						"FULL" : "HALF");
			}
			break;
		case PHY_NOLINK:
			err = phy_read_status(phydev);

			if (err)
				break;

			if (phydev->link) {
				phydev->state = PHY_RUNNING;
				netif_carrier_on(phydev->attached_dev);
				phydev->adjust_link(phydev->attached_dev);
			}
			break;
		case PHY_FORCING:
			err = genphy_update_link(phydev);

			if (err)
				break;

			if (phydev->link) {
				phydev->state = PHY_RUNNING;
				netif_carrier_on(phydev->attached_dev);
			} else {
				if (0 == phydev->link_timeout--) {
					phy_force_reduction(phydev);
					needs_aneg = 1;
				}
			}

			phydev->adjust_link(phydev->attached_dev);
			break;
		case PHY_RUNNING:
			/* Only register a CHANGE if we are
			 * polling */
			if (PHY_POLL == phydev->irq)
				phydev->state = PHY_CHANGELINK;
			break;
		case PHY_CHANGELINK:
			err = phy_read_status(phydev);

			if (err)
				break;

			if (phydev->link) {
				phydev->state = PHY_RUNNING;
				netif_carrier_on(phydev->attached_dev);
			} else {
				phydev->state = PHY_NOLINK;
				netif_carrier_off(phydev->attached_dev);
			}

			phydev->adjust_link(phydev->attached_dev);

			if (PHY_POLL != phydev->irq)
				err = phy_config_interrupt(phydev,
						PHY_INTERRUPT_ENABLED);
			break;
		case PHY_HALTED:
			if (phydev->link) {
				phydev->link = 0;
				netif_carrier_off(phydev->attached_dev);
				phydev->adjust_link(phydev->attached_dev);
			}
			break;
		case PHY_RESUMING:

			err = phy_clear_interrupt(phydev);

			if (err)
				break;

			err = phy_config_interrupt(phydev,
					PHY_INTERRUPT_ENABLED);

			if (err)
				break;

			if (AUTONEG_ENABLE == phydev->autoneg) {
				err = phy_aneg_done(phydev);
				if (err < 0)
					break;

				/* err > 0 if AN is done.
				 * Otherwise, it's 0, and we're
				 * still waiting for AN */
				if (err > 0) {
					err = phy_read_status(phydev);
					if (err)
						break;

					if (phydev->link) {
						phydev->state = PHY_RUNNING;
						netif_carrier_on(phydev->attached_dev);
					} else
						phydev->state = PHY_NOLINK;
					phydev->adjust_link(phydev->attached_dev);
				} else {
					phydev->state = PHY_AN;
					phydev->link_timeout = PHY_AN_TIMEOUT;
				}
			} else {
				err = phy_read_status(phydev);
				if (err)
					break;

				if (phydev->link) {
					phydev->state = PHY_RUNNING;
					netif_carrier_on(phydev->attached_dev);
				} else
					phydev->state = PHY_NOLINK;
				phydev->adjust_link(phydev->attached_dev);
			}
			break;
	}

	mutex_unlock(&phydev->lock);

	if (needs_aneg)
		err = phy_start_aneg(phydev);

	if (err < 0)
		phy_error(phydev);

	schedule_delayed_work(&phydev->state_queue, PHY_STATE_TIME * HZ);
}
