

#include <linux/sys.h>
#include <linux/linkage.h>
#include <asm/errno.h>
#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/asm-offsets.h>
#include <asm/entry.h>
#include <asm/unistd.h>

.text

.globl buserr
.globl trap
.globl ret_from_exception
.globl ret_from_signal
.globl sys_fork
.globl sys_clone
.globl sys_vfork

ENTRY(buserr)
	SAVE_ALL
	moveq	#-1,%d0
	movel	%d0,%sp@(PT_OFF_ORIG_D0)
	movel	%sp,%sp@- 		/* stack frame pointer argument */
	jsr	buserr_c
	addql	#4,%sp
	jra	ret_from_exception

ENTRY(trap)
	SAVE_ALL
	moveq	#-1,%d0
	movel	%d0,%sp@(PT_OFF_ORIG_D0)
	movel	%sp,%sp@- 		/* stack frame pointer argument */
	jsr	trap_c
	addql	#4,%sp
	jra	ret_from_exception

#ifdef TRAP_DBG_INTERRUPT

.globl dbginterrupt
ENTRY(dbginterrupt)
	SAVE_ALL
	moveq	#-1,%d0
	movel	%d0,%sp@(PT_OFF_ORIG_D0)
	movel	%sp,%sp@- 		/* stack frame pointer argument */
	jsr	dbginterrupt_c
	addql	#4,%sp
	jra	ret_from_exception
#endif

ENTRY(reschedule)
	/* save top of frame */
	pea	%sp@
	jbsr	set_esp0
	addql	#4,%sp
	pea	ret_from_exception
	jmp	schedule

ENTRY(ret_from_fork)
	movel	%d1,%sp@-
	jsr	schedule_tail
	addql	#4,%sp
	jra	ret_from_exception

ENTRY(sys_fork)
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jbsr	m68k_fork
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_vfork)
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jbsr	m68k_vfork
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_clone)
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jbsr	m68k_clone
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_sigsuspend)
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jbsr	do_sigsuspend
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_rt_sigsuspend)
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jbsr	do_rt_sigsuspend
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_sigreturn)
	SAVE_SWITCH_STACK
	jbsr	do_sigreturn
	RESTORE_SWITCH_STACK
	rts

ENTRY(sys_rt_sigreturn)
	SAVE_SWITCH_STACK
	jbsr	do_rt_sigreturn
	RESTORE_SWITCH_STACK
	rts

ENTRY(ret_from_user_signal)
	moveq #__NR_sigreturn,%d0
	trap #0

ENTRY(ret_from_user_rt_signal)
	movel #__NR_rt_sigreturn,%d0
	trap #0

