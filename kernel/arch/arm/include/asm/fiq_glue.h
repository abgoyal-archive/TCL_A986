

#ifndef __ASM_FIQ_GLUE_H
#define __ASM_FIQ_GLUE_H

struct fiq_glue_handler {
	void (*fiq)(struct fiq_glue_handler *h, void *regs, void *svc_sp);
	void (*resume)(struct fiq_glue_handler *h);
};

int fiq_glue_register_handler(struct fiq_glue_handler *handler);

#ifdef CONFIG_FIQ_GLUE
void fiq_glue_resume(void);
#else
static inline void fiq_glue_resume(void) {}
#endif

#endif
