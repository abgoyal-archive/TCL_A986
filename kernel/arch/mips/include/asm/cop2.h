
#ifndef __ASM_COP2_H
#define __ASM_COP2_H

enum cu2_ops {
	CU2_EXCEPTION,
	CU2_LWC2_OP,
	CU2_LDC2_OP,
	CU2_SWC2_OP,
	CU2_SDC2_OP,
};

extern int register_cu2_notifier(struct notifier_block *nb);
extern int cu2_notifier_call_chain(unsigned long val, void *v);

#endif /* __ASM_COP2_H */
