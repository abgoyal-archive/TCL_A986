
#ifndef __ASM_SH_DREAMCAST_SYSASIC_H
#define __ASM_SH_DREAMCAST_SYSASIC_H

#include <asm/irq.h>


#define HW_EVENT_IRQ_BASE  48

/* IRQ 13 */
#define HW_EVENT_VSYNC     (HW_EVENT_IRQ_BASE +  5) /* VSync */
#define HW_EVENT_MAPLE_DMA (HW_EVENT_IRQ_BASE + 12) /* Maple DMA complete */
#define HW_EVENT_GDROM_DMA (HW_EVENT_IRQ_BASE + 14) /* GD-ROM DMA complete */
#define HW_EVENT_G2_DMA    (HW_EVENT_IRQ_BASE + 15) /* G2 DMA complete */
#define HW_EVENT_PVR2_DMA  (HW_EVENT_IRQ_BASE + 19) /* PVR2 DMA complete */

/* IRQ 11 */
#define HW_EVENT_GDROM_CMD (HW_EVENT_IRQ_BASE + 32) /* GD-ROM cmd. complete */
#define HW_EVENT_AICA_SYS  (HW_EVENT_IRQ_BASE + 33) /* AICA-related */
#define HW_EVENT_EXTERNAL  (HW_EVENT_IRQ_BASE + 35) /* Ext. (expansion) */

#define HW_EVENT_IRQ_MAX (HW_EVENT_IRQ_BASE + 95)

/* arch/sh/boards/mach-dreamcast/irq.c */
extern int systemasic_irq_demux(int);
extern void systemasic_irq_init(void);
extern void aica_time_init(void);

#endif /* __ASM_SH_DREAMCAST_SYSASIC_H */

