

#ifndef _POWERPC_KERNEL_XICS_H
#define _POWERPC_KERNEL_XICS_H

extern void xics_init_IRQ(void);
extern void xics_setup_cpu(void);
extern void xics_teardown_cpu(void);
extern void xics_kexec_teardown_cpu(int secondary);
extern void xics_migrate_irqs_away(void);
extern int smp_xics_probe(void);
extern void smp_xics_message_pass(int target, int msg);

#endif /* _POWERPC_KERNEL_XICS_H */
