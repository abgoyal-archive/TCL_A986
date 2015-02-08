

#define VDSO_PRELINK 0
#include "../vdso-layout.lds.S"

/* The ELF entry point can be used to set the AT_SYSINFO value.  */
ENTRY(__kernel_vsyscall);

VERSION
{
	LINUX_2.5 {
	global:
		__kernel_vsyscall;
		__kernel_sigreturn;
		__kernel_rt_sigreturn;
	local: *;
	};
}

VDSO32_PRELINK		= VDSO_PRELINK;
VDSO32_vsyscall		= __kernel_vsyscall;
VDSO32_sigreturn	= __kernel_sigreturn;
VDSO32_rt_sigreturn	= __kernel_rt_sigreturn;
