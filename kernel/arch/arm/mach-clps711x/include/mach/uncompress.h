
#include <mach/io.h>
#include <mach/hardware.h>
#include <asm/hardware/clps7111.h>

#undef CLPS7111_BASE
#define CLPS7111_BASE CLPS7111_PHYS_BASE

#define __raw_readl(p)		(*(unsigned long *)(p))
#define __raw_writel(v,p)	(*(unsigned long *)(p) = (v))

#ifdef CONFIG_DEBUG_CLPS711X_UART2
#define SYSFLGx	SYSFLG2
#define UARTDRx	UARTDR2
#else
#define SYSFLGx	SYSFLG1
#define UARTDRx	UARTDR1
#endif

static inline void putc(int c)
{
	while (clps_readl(SYSFLGx) & SYSFLG_UTXFF)
		barrier();
	clps_writel(c, UARTDRx);
}

static inline void flush(void)
{
	while (clps_readl(SYSFLGx) & SYSFLG_UBUSY)
		barrier();
}

#define arch_decomp_setup()

#define arch_decomp_wdog()
