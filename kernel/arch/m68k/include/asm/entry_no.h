
#ifndef __M68KNOMMU_ENTRY_H
#define __M68KNOMMU_ENTRY_H

#include <asm/setup.h>
#include <asm/page.h>


#define ALLOWINT 0xf8ff

#ifdef __ASSEMBLY__

/* process bits for task_struct.flags */
PF_TRACESYS_OFF = 3
PF_TRACESYS_BIT = 5
PF_PTRACED_OFF = 3
PF_PTRACED_BIT = 4
PF_DTRACE_OFF = 1
PF_DTRACE_BIT = 5

LENOSYS = 38

#define SWITCH_STACK_SIZE (6*4+4)	/* Includes return address */


#ifdef CONFIG_COLDFIRE
.macro SAVE_ALL
	move	#0x2700,%sr		/* disable intrs */
	btst	#5,%sp@(2)		/* from user? */
	bnes	6f			/* no, skip */
	movel	%sp,sw_usp		/* save user sp */
	addql	#8,sw_usp		/* remove exception */
	movel	sw_ksp,%sp		/* kernel sp */
	subql	#8,%sp			/* room for exception */
	clrl	%sp@-			/* stkadj */
	movel	%d0,%sp@-		/* orig d0 */
	movel	%d0,%sp@-		/* d0 */
	lea	%sp@(-32),%sp		/* space for 8 regs */
	moveml	%d1-%d5/%a0-%a2,%sp@
	movel	sw_usp,%a0		/* get usp */
	movel	%a0@-,%sp@(PT_OFF_PC)	/* copy exception program counter */
	movel	%a0@-,%sp@(PT_OFF_FORMATVEC)/*copy exception format/vector/sr */
	bra	7f
	6:
	clrl	%sp@-			/* stkadj */
	movel	%d0,%sp@-		/* orig d0 */
	movel	%d0,%sp@-		/* d0 */
	lea	%sp@(-32),%sp		/* space for 8 regs */
	moveml	%d1-%d5/%a0-%a2,%sp@
	7:
.endm

.macro RESTORE_ALL
	btst	#5,%sp@(PT_SR)		/* going user? */
	bnes	8f			/* no, skip */
	move	#0x2700,%sr		/* disable intrs */
	movel	sw_usp,%a0		/* get usp */
	movel	%sp@(PT_OFF_PC),%a0@-	/* copy exception program counter */
	movel	%sp@(PT_OFF_FORMATVEC),%a0@-/*copy exception format/vector/sr */
	moveml	%sp@,%d1-%d5/%a0-%a2
	lea	%sp@(32),%sp		/* space for 8 regs */
	movel	%sp@+,%d0
	addql	#4,%sp			/* orig d0 */
	addl	%sp@+,%sp		/* stkadj */
	addql	#8,%sp			/* remove exception */
	movel	%sp,sw_ksp		/* save ksp */
	subql	#8,sw_usp		/* set exception */
	movel	sw_usp,%sp		/* restore usp */
	rte
	8:
	moveml	%sp@,%d1-%d5/%a0-%a2
	lea	%sp@(32),%sp		/* space for 8 regs */
	movel	%sp@+,%d0
	addql	#4,%sp			/* orig d0 */
	addl	%sp@+,%sp		/* stkadj */
	rte
.endm

.macro SAVE_LOCAL
	move	#0x2700,%sr		/* disable intrs */
	clrl	%sp@-			/* stkadj */
	movel	%d0,%sp@-		/* orig d0 */
	movel	%d0,%sp@-		/* d0 */
	lea	%sp@(-32),%sp		/* space for 8 regs */
	moveml	%d1-%d5/%a0-%a2,%sp@
.endm

.macro RESTORE_LOCAL
	moveml	%sp@,%d1-%d5/%a0-%a2
	lea	%sp@(32),%sp		/* space for 8 regs */
	movel	%sp@+,%d0
	addql	#4,%sp			/* orig d0 */
	addl	%sp@+,%sp		/* stkadj */
	rte
.endm

.macro SAVE_SWITCH_STACK
	lea	%sp@(-24),%sp		/* 6 regs */
	moveml	%a3-%a6/%d6-%d7,%sp@
.endm

.macro RESTORE_SWITCH_STACK
	moveml	%sp@,%a3-%a6/%d6-%d7
	lea	%sp@(24),%sp		/* 6 regs */
.endm

.globl sw_usp
.globl sw_ksp

#else /* !CONFIG_COLDFIRE */

.macro SAVE_ALL
	clrl	%sp@-			/* stkadj */
	movel	%d0,%sp@-		/* orig d0 */
	movel	%d0,%sp@-		/* d0 */
	moveml	%d1-%d5/%a0-%a2,%sp@-
.endm

.macro RESTORE_ALL
	moveml	%sp@+,%a0-%a2/%d1-%d5
	movel	%sp@+,%d0
	addql	#4,%sp			/* orig d0 */
	addl	%sp@+,%sp		/* stkadj */
	rte
.endm

.macro SAVE_SWITCH_STACK
	moveml	%a3-%a6/%d6-%d7,%sp@-
.endm

.macro RESTORE_SWITCH_STACK
	moveml	%sp@+,%a3-%a6/%d6-%d7
.endm

#endif /* !CONFIG_COLDFIRE */
#endif /* __ASSEMBLY__ */
#endif /* __M68KNOMMU_ENTRY_H */
