

#ifndef __ASM_ARM_ARCH_OMAP_PRCM_H
#define __ASM_ARM_ARCH_OMAP_PRCM_H

u32 omap_prcm_get_reset_sources(void);
void omap_prcm_arch_reset(char mode, const char *cmd);
int omap2_cm_wait_idlest(void __iomem *reg, u32 mask, u8 idlest,
			 const char *name);

#define START_PADCONF_SAVE 0x2
#define PADCONF_SAVE_DONE  0x1

void omap3_prcm_save_context(void);
void omap3_prcm_restore_context(void);

u32 prm_read_mod_reg(s16 module, u16 idx);
void prm_write_mod_reg(u32 val, s16 module, u16 idx);
u32 prm_rmw_mod_reg_bits(u32 mask, u32 bits, s16 module, s16 idx);
u32 prm_read_mod_bits_shift(s16 domain, s16 idx, u32 mask);
u32 cm_read_mod_reg(s16 module, u16 idx);
void cm_write_mod_reg(u32 val, s16 module, u16 idx);
u32 cm_rmw_mod_reg_bits(u32 mask, u32 bits, s16 module, s16 idx);

#endif



