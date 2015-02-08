


#include "vpx_ports/config.h"
#if ARCH_X86 || ARCH_X86_64
void vpx_reset_mmx_state(void);
#define vp8_clear_system_state() vpx_reset_mmx_state()
#else
#define vp8_clear_system_state()
#endif

struct VP8Common;
void vp8_machine_specific_config(struct VP8Common *);
