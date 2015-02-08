
#ifndef __ASM_ARCH_CPU_H
#define __ASM_ARCH_CPU_H

#ifdef CONFIG_CPU_AT32AP700X
# define cpu_is_at32ap7000()	(1)
#else
# define cpu_is_at32ap7000()	(0)
#endif

#define cpu_is_at91rm9200()	(0)
#define cpu_is_at91sam9xe()	(0)
#define cpu_is_at91sam9260()	(0)
#define cpu_is_at91sam9261()	(0)
#define cpu_is_at91sam9263()	(0)
#define cpu_is_at91sam9rl()	(0)
#define cpu_is_at91cap9()	(0)
#define cpu_is_at91sam9g10()	(0)
#define cpu_is_at91sam9g45()	(0)
#define cpu_is_at91sam9g45es()	(0)

#endif /* __ASM_ARCH_CPU_H */
