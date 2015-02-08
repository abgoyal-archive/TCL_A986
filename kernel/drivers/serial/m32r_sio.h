


struct m32r_sio_probe {
	struct module	*owner;
	int		(*pci_init_one)(struct pci_dev *dev);
	void		(*pci_remove_one)(struct pci_dev *dev);
	void		(*pnp_init)(void);
};

int m32r_sio_register_probe(struct m32r_sio_probe *probe);
void m32r_sio_unregister_probe(struct m32r_sio_probe *probe);
void m32r_sio_get_irq_map(unsigned int *map);
void m32r_sio_suspend_port(int line);
void m32r_sio_resume_port(int line);

struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	unsigned int flags;
	unsigned char io_type;
	unsigned char __iomem *iomem_base;
	unsigned short iomem_reg_shift;
};

#define _INLINE_ inline

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)
