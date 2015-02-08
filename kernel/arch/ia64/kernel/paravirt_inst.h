

#ifdef __IA64_ASM_PARAVIRTUALIZED_PVCHECK
#include <asm/native/pvchk_inst.h>
#elif defined(__IA64_ASM_PARAVIRTUALIZED_XEN)
#include <asm/xen/inst.h>
#include <asm/xen/minstate.h>
#else
#include <asm/native/inst.h>
#endif

