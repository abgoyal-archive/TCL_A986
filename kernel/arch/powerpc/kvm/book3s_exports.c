

#include <linux/module.h>
#include <asm/kvm_book3s.h>

EXPORT_SYMBOL_GPL(kvmppc_trampoline_enter);
EXPORT_SYMBOL_GPL(kvmppc_trampoline_lowmem);
EXPORT_SYMBOL_GPL(kvmppc_rmcall);
EXPORT_SYMBOL_GPL(kvmppc_load_up_fpu);
#ifdef CONFIG_ALTIVEC
EXPORT_SYMBOL_GPL(kvmppc_load_up_altivec);
#endif
#ifdef CONFIG_VSX
EXPORT_SYMBOL_GPL(kvmppc_load_up_vsx);
#endif
