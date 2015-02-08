
#ifndef __ASM_PB1550_H
#define __ASM_PB1550_H

#include <linux/types.h>
#include <asm/mach-au1x00/au1xxx_psc.h>

#define DBDMA_AC97_TX_CHAN	DSCR_CMD0_PSC1_TX
#define DBDMA_AC97_RX_CHAN	DSCR_CMD0_PSC1_RX
#define DBDMA_I2S_TX_CHAN	DSCR_CMD0_PSC3_TX
#define DBDMA_I2S_RX_CHAN	DSCR_CMD0_PSC3_RX

#define SPI_PSC_BASE		PSC0_BASE_ADDR
#define AC97_PSC_BASE		PSC1_BASE_ADDR
#define SMBUS_PSC_BASE		PSC2_BASE_ADDR
#define I2S_PSC_BASE		PSC3_BASE_ADDR

#if defined(CONFIG_MTD_PB1550_BOOT) && defined(CONFIG_MTD_PB1550_USER)
#define PB1550_BOTH_BANKS
#elif defined(CONFIG_MTD_PB1550_BOOT) && !defined(CONFIG_MTD_PB1550_USER)
#define PB1550_BOOT_ONLY
#elif !defined(CONFIG_MTD_PB1550_BOOT) && defined(CONFIG_MTD_PB1550_USER)
#define PB1550_USER_ONLY
#endif

#define NAND_T_H		(18 >> 2)
#define NAND_T_PUL		(30 >> 2)
#define NAND_T_SU		(30 >> 2)
#define NAND_T_WH		(30 >> 2)

/* Bitfield shift amounts */
#define NAND_T_H_SHIFT		0
#define NAND_T_PUL_SHIFT	4
#define NAND_T_SU_SHIFT		8
#define NAND_T_WH_SHIFT		12

#define NAND_TIMING	(((NAND_T_H   & 0xF) << NAND_T_H_SHIFT)   | \
			 ((NAND_T_PUL & 0xF) << NAND_T_PUL_SHIFT) | \
			 ((NAND_T_SU  & 0xF) << NAND_T_SU_SHIFT)  | \
			 ((NAND_T_WH  & 0xF) << NAND_T_WH_SHIFT))

#define NAND_CS 1

/* Should be done by YAMON */
#define NAND_STCFG	0x00400005 /* 8-bit NAND */
#define NAND_STTIME	0x00007774 /* valid for 396 MHz SD=2 only */
#define NAND_STADDR	0x12000FFF /* physical address 0x20000000 */

#endif /* __ASM_PB1550_H */
