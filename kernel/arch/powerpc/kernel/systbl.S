

#include <asm/ppc_asm.h>

#ifdef CONFIG_PPC64
#define SYSCALL(func)		.llong	.sys_##func,.sys_##func
#define COMPAT_SYS(func)	.llong	.sys_##func,.compat_sys_##func
#define PPC_SYS(func)		.llong	.ppc_##func,.ppc_##func
#define OLDSYS(func)		.llong	.sys_ni_syscall,.sys_ni_syscall
#define SYS32ONLY(func)		.llong	.sys_ni_syscall,.compat_sys_##func
#define SYSX(f, f3264, f32)	.llong	.f,.f3264
#else
#define SYSCALL(func)		.long	sys_##func
#define COMPAT_SYS(func)	.long	sys_##func
#define PPC_SYS(func)		.long	ppc_##func
#define OLDSYS(func)		.long	sys_##func
#define SYS32ONLY(func)		.long	sys_##func
#define SYSX(f, f3264, f32)	.long	f32
#endif
#define SYSCALL_SPU(func)	SYSCALL(func)
#define COMPAT_SYS_SPU(func)	COMPAT_SYS(func)
#define PPC_SYS_SPU(func)	PPC_SYS(func)
#define SYSX_SPU(f, f3264, f32)	SYSX(f, f3264, f32)

#ifdef CONFIG_PPC64
#define sys_sigpending	sys_ni_syscall
#define sys_old_getrlimit sys_ni_syscall

	.p2align	3
#endif

_GLOBAL(sys_call_table)
#include <asm/systbl.h>
