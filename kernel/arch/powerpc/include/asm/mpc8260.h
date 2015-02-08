
#ifdef __KERNEL__
#ifndef __ASM_POWERPC_MPC8260_H__
#define __ASM_POWERPC_MPC8260_H__

#define MPC82XX_BCR_PLDP 0x00800000 /* Pipeline Maximum Depth */

#ifdef CONFIG_8260

#if defined(CONFIG_PQ2ADS) || defined (CONFIG_PQ2FADS)
#include <platforms/82xx/pq2ads.h>
#endif

#ifdef CONFIG_PCI_8260
#include <platforms/82xx/m82xx_pci.h>
#endif

#endif /* CONFIG_8260 */
#endif /* !__ASM_POWERPC_MPC8260_H__ */
#endif /* __KERNEL__ */
