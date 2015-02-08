
#include <linux/module.h>

#include <asm/bootinfo.h>

#include <loongson.h>

unsigned long cpu_clock_freq;
EXPORT_SYMBOL(cpu_clock_freq);
unsigned long memsize, highmemsize;

#define parse_even_earlier(res, option, p)				\
do {									\
	if (strncmp(option, (char *)p, strlen(option)) == 0)		\
			strict_strtol((char *)p + strlen(option"="),	\
					10, &res);			\
} while (0)

void __init prom_init_env(void)
{
	/* pmon passes arguments in 32bit pointers */
	int *_prom_envp;
	unsigned long bus_clock;
	unsigned int processor_id;
	long l;

	/* firmware arguments are initialized in head.S */
	_prom_envp = (int *)fw_arg2;

	l = (long)*_prom_envp;
	while (l != 0) {
		parse_even_earlier(bus_clock, "busclock", l);
		parse_even_earlier(cpu_clock_freq, "cpuclock", l);
		parse_even_earlier(memsize, "memsize", l);
		parse_even_earlier(highmemsize, "highmemsize", l);
		_prom_envp++;
		l = (long)*_prom_envp;
	}
	if (memsize == 0)
		memsize = 256;
	if (bus_clock == 0)
		bus_clock = 66000000;
	if (cpu_clock_freq == 0) {
		processor_id = (&current_cpu_data)->processor_id;
		switch (processor_id & PRID_REV_MASK) {
		case PRID_REV_LOONGSON2E:
			cpu_clock_freq = 533080000;
			break;
		case PRID_REV_LOONGSON2F:
			cpu_clock_freq = 797000000;
			break;
		default:
			cpu_clock_freq = 100000000;
			break;
		}
	}

	pr_info("busclock=%ld, cpuclock=%ld, memsize=%ld, highmemsize=%ld\n",
		bus_clock, cpu_clock_freq, memsize, highmemsize);
}
