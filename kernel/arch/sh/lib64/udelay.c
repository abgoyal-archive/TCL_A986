
#include <linux/sched.h>
#include <asm/param.h>


void __delay(unsigned long loops)
{
	long long dummy;
	__asm__ __volatile__("gettr	tr0, %1\n\t"
			     "pta	$+4, tr0\n\t"
			     "addi	%0, -1, %0\n\t"
			     "bne	%0, r63, tr0\n\t"
			     "ptabs	%1, tr0\n\t":"=r"(loops),
			     "=r"(dummy)
			     :"0"(loops));
}

void __const_udelay(unsigned long xloops)
{
	__delay(xloops * (HZ * cpu_data[raw_smp_processor_id()].loops_per_jiffy));
}

void __udelay(unsigned long usecs)
{
	__const_udelay(usecs * 0x000010c6);  /* 2**32 / 1000000 */
}

void __ndelay(unsigned long nsecs)
{
	__const_udelay(nsecs * 0x00000005);
}
