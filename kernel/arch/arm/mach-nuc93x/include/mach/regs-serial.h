

#ifndef __ASM_ARM_REGS_SERIAL_H
#define __ASM_ARM_REGS_SERIAL_H

#define UART0_BA	NUC93X_VA_UART
#define UART1_BA	(NUC93X_VA_UART+0x100)

#define UART0_PA	NUC93X_PA_UART
#define UART1_PA	(NUC93X_PA_UART+0x100)


#ifndef __ASSEMBLY__

struct nuc93x_uart_clksrc {
	const char	*name;
	unsigned int	divisor;
	unsigned int	min_baud;
	unsigned int	max_baud;
};

struct nuc93x_uartcfg {
	unsigned char	hwport;
	unsigned char	unused;
	unsigned short	flags;
	unsigned long	uart_flags;

	unsigned long	ucon;
	unsigned long	ulcon;
	unsigned long	ufcon;

	struct nuc93x_uart_clksrc *clocks;
	unsigned int	clocks_size;
};

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_REGS_SERIAL_H */

