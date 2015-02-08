
#include <linux/module.h>
#include <linux/param.h>
#include <linux/smp.h>

#include <asm/compiler.h>
#include <asm/war.h>

inline void __delay(unsigned int loops)
{
	__asm__ __volatile__ (
	"	.set	noreorder				\n"
	"	.align	3					\n"
	"1:	bnez	%0, 1b					\n"
	"	subu	%0, 1					\n"
	"	.set	reorder					\n"
	: "=r" (loops)
	: "0" (loops));
}
EXPORT_SYMBOL(__delay);


void __udelay(unsigned long us)
{
	unsigned int lpj = raw_current_cpu_data.udelay_val;

	__delay((us * 0x000010c7ull * HZ * lpj) >> 32);
}
EXPORT_SYMBOL(__udelay);

void __ndelay(unsigned long ns)
{
	unsigned int lpj = raw_current_cpu_data.udelay_val;

	__delay((ns * 0x00000005ull * HZ * lpj) >> 32);
}
EXPORT_SYMBOL(__ndelay);
