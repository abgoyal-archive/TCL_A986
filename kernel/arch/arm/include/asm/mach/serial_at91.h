

struct uart_port;

struct atmel_port_fns {
	void	(*set_mctrl)(struct uart_port *, u_int);
	u_int	(*get_mctrl)(struct uart_port *);
	void	(*enable_ms)(struct uart_port *);
	void	(*pm)(struct uart_port *, u_int, u_int);
	int	(*set_wake)(struct uart_port *, u_int);
	int	(*open)(struct uart_port *);
	void	(*close)(struct uart_port *);
};

#if defined(CONFIG_SERIAL_ATMEL)
void atmel_register_uart_fns(struct atmel_port_fns *fns);
#else
#define atmel_register_uart_fns(fns) do { } while (0)
#endif


