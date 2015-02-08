
#ifndef __ASM_ARCH_DM644X_H
#define __ASM_ARCH_DM644X_H

#include <linux/davinci_emac.h>
#include <mach/hardware.h>
#include <mach/asp.h>
#include <media/davinci/vpfe_capture.h>

#define DM644X_EMAC_BASE		(0x01C80000)
#define DM644X_EMAC_CNTRL_OFFSET	(0x0000)
#define DM644X_EMAC_CNTRL_MOD_OFFSET	(0x1000)
#define DM644X_EMAC_CNTRL_RAM_OFFSET	(0x2000)
#define DM644X_EMAC_MDIO_OFFSET		(0x4000)
#define DM644X_EMAC_CNTRL_RAM_SIZE	(0x2000)

#define DM644X_ASYNC_EMIF_CONTROL_BASE	0x01E00000
#define DM644X_ASYNC_EMIF_DATA_CE0_BASE 0x02000000
#define DM644X_ASYNC_EMIF_DATA_CE1_BASE 0x04000000
#define DM644X_ASYNC_EMIF_DATA_CE2_BASE 0x06000000
#define DM644X_ASYNC_EMIF_DATA_CE3_BASE 0x08000000

void __init dm644x_init(void);
void __init dm644x_init_asp(struct snd_platform_data *pdata);
void dm644x_set_vpfe_config(struct vpfe_config *cfg);

#endif /* __ASM_ARCH_DM644X_H */
