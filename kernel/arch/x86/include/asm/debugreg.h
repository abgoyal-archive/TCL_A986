
#ifndef _ASM_X86_DEBUGREG_H
#define _ASM_X86_DEBUGREG_H


#define DR_FIRSTADDR 0        /* u_debugreg[DR_FIRSTADDR] */
#define DR_LASTADDR 3         /* u_debugreg[DR_LASTADDR]  */

#define DR_STATUS 6           /* u_debugreg[DR_STATUS]     */
#define DR_CONTROL 7          /* u_debugreg[DR_CONTROL] */


/* Define reserved bits in DR6 which are always set to 1 */
#define DR6_RESERVED	(0xFFFF0FF0)

#define DR_TRAP0	(0x1)		/* db0 */
#define DR_TRAP1	(0x2)		/* db1 */
#define DR_TRAP2	(0x4)		/* db2 */
#define DR_TRAP3	(0x8)		/* db3 */
#define DR_TRAP_BITS	(DR_TRAP0|DR_TRAP1|DR_TRAP2|DR_TRAP3)

#define DR_STEP		(0x4000)	/* single-step */
#define DR_SWITCH	(0x8000)	/* task switch */


#define DR_CONTROL_SHIFT 16 /* Skip this many bits in ctl register */
#define DR_CONTROL_SIZE 4   /* 4 control bits per register */

#define DR_RW_EXECUTE (0x0)   /* Settings for the access types to trap on */
#define DR_RW_WRITE (0x1)
#define DR_RW_READ (0x3)

#define DR_LEN_1 (0x0) /* Settings for data length to trap on */
#define DR_LEN_2 (0x4)
#define DR_LEN_4 (0xC)
#define DR_LEN_8 (0x8)


#define DR_LOCAL_ENABLE_SHIFT 0    /* Extra shift to the local enable bit */
#define DR_GLOBAL_ENABLE_SHIFT 1   /* Extra shift to the global enable bit */
#define DR_LOCAL_ENABLE (0x1)      /* Local enable for reg 0 */
#define DR_GLOBAL_ENABLE (0x2)     /* Global enable for reg 0 */
#define DR_ENABLE_SIZE 2           /* 2 enable bits per register */

#define DR_LOCAL_ENABLE_MASK (0x55)  /* Set  local bits for all 4 regs */
#define DR_GLOBAL_ENABLE_MASK (0xAA) /* Set global bits for all 4 regs */


#ifdef __i386__
#define DR_CONTROL_RESERVED (0xFC00) /* Reserved by Intel */
#else
#define DR_CONTROL_RESERVED (0xFFFFFFFF0000FC00UL) /* Reserved */
#endif

#define DR_LOCAL_SLOWDOWN (0x100)   /* Local slow the pipeline */
#define DR_GLOBAL_SLOWDOWN (0x200)  /* Global slow the pipeline */

#ifdef __KERNEL__

DECLARE_PER_CPU(unsigned long, cpu_dr7);

static inline void hw_breakpoint_disable(void)
{
	/* Zero the control register for HW Breakpoint */
	set_debugreg(0UL, 7);

	/* Zero-out the individual HW breakpoint address registers */
	set_debugreg(0UL, 0);
	set_debugreg(0UL, 1);
	set_debugreg(0UL, 2);
	set_debugreg(0UL, 3);
}

static inline int hw_breakpoint_active(void)
{
	return __get_cpu_var(cpu_dr7) & DR_GLOBAL_ENABLE_MASK;
}

extern void aout_dump_debugregs(struct user *dump);

extern void hw_breakpoint_restore(void);

#endif	/* __KERNEL__ */

#endif /* _ASM_X86_DEBUGREG_H */
