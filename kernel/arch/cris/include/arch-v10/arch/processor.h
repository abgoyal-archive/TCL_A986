
#ifndef __ASM_CRIS_ARCH_PROCESSOR_H
#define __ASM_CRIS_ARCH_PROCESSOR_H

#define current_text_addr() ({void *pc; __asm__ ("move.d $pc,%0" : "=rm" (pc)); pc; })

/* CRIS has no problems with write protection */
#define wp_works_ok 1


struct thread_struct {
	unsigned long ksp;     /* kernel stack pointer */
	unsigned long usp;     /* user stack pointer */
	unsigned long dccr;    /* saved flag register */
};


#ifdef CONFIG_CRIS_LOW_MAP
#define TASK_SIZE       (0x50000000UL)   /* 1.25 GB */
#else
#define TASK_SIZE       (0xA0000000UL)   /* 2.56 GB */
#endif

#define INIT_THREAD  { \
   0, 0, 0x20 }  /* ccr = int enable, nothing else */

#define KSTK_EIP(tsk)	\
({			\
	unsigned long eip = 0;   \
	unsigned long regs = (unsigned long)task_pt_regs(tsk); \
	if (regs > PAGE_SIZE && \
		virt_addr_valid(regs)) \
	eip = ((struct pt_regs *)regs)->irp; \
	eip; \
})


#define start_thread(regs, ip, usp) do { \
	set_fs(USER_DS);      \
	regs->irp = ip;       \
	regs->dccr |= 1 << U_DCCR_BITNR; \
	wrusp(usp);           \
} while(0)

#define arch_fixup(regs) \
   regs->frametype = CRIS_FRAME_NORMAL;

#endif
