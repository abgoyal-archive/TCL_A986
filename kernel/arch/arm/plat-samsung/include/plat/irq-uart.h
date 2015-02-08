

struct s3c_uart_irq {
	void __iomem	*regs;
	unsigned int	 base_irq;
	unsigned int	 parent_irq;
};

extern void s3c_init_uart_irqs(struct s3c_uart_irq *irq, unsigned int nr_irqs);

