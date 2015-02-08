

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
#include <asm/system.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/ioport.h>

#include "et1310_phy.h"
#include "et1310_rx.h"
#include "et131x_adapter.h"
#include "et131x.h"

void EnablePhyComa(struct et131x_adapter *etdev)
{
	unsigned long flags;
	u32 GlobalPmCSR;

	GlobalPmCSR = readl(&etdev->regs->global.pm_csr);

	/* Save the GbE PHY speed and duplex modes. Need to restore this
	 * when cable is plugged back in
	 */
	etdev->PoMgmt.PowerDownSpeed = etdev->AiForceSpeed;
	etdev->PoMgmt.PowerDownDuplex = etdev->AiForceDpx;

	/* Stop sending packets. */
	spin_lock_irqsave(&etdev->SendHWLock, flags);
	etdev->Flags |= fMP_ADAPTER_LOWER_POWER;
	spin_unlock_irqrestore(&etdev->SendHWLock, flags);

	/* Wait for outstanding Receive packets */

	/* Gate off JAGCore 3 clock domains */
	GlobalPmCSR &= ~ET_PMCSR_INIT;
	writel(GlobalPmCSR, &etdev->regs->global.pm_csr);

	/* Program gigE PHY in to Coma mode */
	GlobalPmCSR |= ET_PM_PHY_SW_COMA;
	writel(GlobalPmCSR, &etdev->regs->global.pm_csr);
}

void DisablePhyComa(struct et131x_adapter *etdev)
{
	u32 GlobalPmCSR;

	GlobalPmCSR = readl(&etdev->regs->global.pm_csr);

	/* Disable phy_sw_coma register and re-enable JAGCore clocks */
	GlobalPmCSR |= ET_PMCSR_INIT;
	GlobalPmCSR &= ~ET_PM_PHY_SW_COMA;
	writel(GlobalPmCSR, &etdev->regs->global.pm_csr);

	/* Restore the GbE PHY speed and duplex modes;
	 * Reset JAGCore; re-configure and initialize JAGCore and gigE PHY
	 */
	etdev->AiForceSpeed = etdev->PoMgmt.PowerDownSpeed;
	etdev->AiForceDpx = etdev->PoMgmt.PowerDownDuplex;

	/* Re-initialize the send structures */
	et131x_init_send(etdev);

	/* Reset the RFD list and re-start RU  */
	et131x_reset_recv(etdev);

	/* Bring the device back to the state it was during init prior to
	 * autonegotiation being complete.  This way, when we get the auto-neg
	 * complete interrupt, we can complete init by calling ConfigMacREGS2.
	 */
	et131x_soft_reset(etdev);

	/* setup et1310 as per the documentation ?? */
	et131x_adapter_setup(etdev);

	/* Allow Tx to restart */
	etdev->Flags &= ~fMP_ADAPTER_LOWER_POWER;

	/* Need to re-enable Rx. */
	et131x_rx_dma_enable(etdev);
}

