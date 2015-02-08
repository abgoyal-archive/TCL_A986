
#ifndef __ASM_SH_BIOS_H
#define __ASM_SH_BIOS_H

#ifdef CONFIG_SH_STANDARD_BIOS

extern void sh_bios_console_write(const char *buf, unsigned int len);
extern void sh_bios_gdb_detach(void);

extern void sh_bios_get_node_addr(unsigned char *node_addr);
extern void sh_bios_shutdown(unsigned int how);

extern void sh_bios_vbr_init(void);
extern void sh_bios_vbr_reload(void);

#else

static inline void sh_bios_vbr_init(void) { }
static inline void sh_bios_vbr_reload(void) { }

#endif /* CONFIG_SH_STANDARD_BIOS */

#endif /* __ASM_SH_BIOS_H */
