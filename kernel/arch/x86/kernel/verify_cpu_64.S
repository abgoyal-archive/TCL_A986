

#include <asm/cpufeature.h>

verify_cpu:
	pushfl				# Save caller passed flags
	pushl	$0			# Kill any dangerous flags
	popfl

	pushfl				# standard way to check for cpuid
	popl	%eax
	movl	%eax,%ebx
	xorl	$0x200000,%eax
	pushl	%eax
	popfl
	pushfl
	popl	%eax
	cmpl	%eax,%ebx
	jz	verify_cpu_no_longmode	# cpu has no cpuid

	movl	$0x0,%eax		# See if cpuid 1 is implemented
	cpuid
	cmpl	$0x1,%eax
	jb	verify_cpu_no_longmode	# no cpuid 1

	xor	%di,%di
	cmpl	$0x68747541,%ebx	# AuthenticAMD
	jnz	verify_cpu_noamd
	cmpl	$0x69746e65,%edx
	jnz	verify_cpu_noamd
	cmpl	$0x444d4163,%ecx
	jnz	verify_cpu_noamd
	mov	$1,%di			# cpu is from AMD

verify_cpu_noamd:
	movl    $0x1,%eax		# Does the cpu have what it takes
	cpuid
	andl	$REQUIRED_MASK0,%edx
	xorl	$REQUIRED_MASK0,%edx
	jnz	verify_cpu_no_longmode

	movl    $0x80000000,%eax	# See if extended cpuid is implemented
	cpuid
	cmpl    $0x80000001,%eax
	jb      verify_cpu_no_longmode	# no extended cpuid

	movl    $0x80000001,%eax	# Does the cpu have what it takes
	cpuid
	andl    $REQUIRED_MASK1,%edx
	xorl    $REQUIRED_MASK1,%edx
	jnz     verify_cpu_no_longmode

verify_cpu_sse_test:
	movl	$1,%eax
	cpuid
	andl	$SSE_MASK,%edx
	cmpl	$SSE_MASK,%edx
	je	verify_cpu_sse_ok
	test	%di,%di
	jz	verify_cpu_no_longmode	# only try to force SSE on AMD
	movl	$0xc0010015,%ecx	# HWCR
	rdmsr
	btr	$15,%eax		# enable SSE
	wrmsr
	xor	%di,%di			# don't loop
	jmp	verify_cpu_sse_test	# try again

verify_cpu_no_longmode:
	popfl				# Restore caller passed flags
	movl $1,%eax
	ret
verify_cpu_sse_ok:
	popfl				# Restore caller passed flags
	xorl %eax, %eax
	ret
