


#ifndef __ASSEMBLY__

static inline unsigned long ixp2000_reg_read(volatile void *reg)
{
	return *((volatile unsigned long *)reg);
}

static inline void ixp2000_reg_write(volatile void *reg, unsigned long val)
{
	*((volatile unsigned long *)reg) = val;
}

static inline void ixp2000_reg_wrb(volatile void *reg, unsigned long val)
{
	unsigned long dummy;

	*((volatile unsigned long *)reg) = val;

	dummy = *((volatile unsigned long *)reg);
	__asm__ __volatile__("mov %0, %0" : "+r" (dummy));
}

struct slowport_cfg {
	unsigned long CCR;	/* Clock divide */
	unsigned long WTC;	/* Write Timing Control */
	unsigned long RTC;	/* Read Timing Control */
	unsigned long PCR;	/* Protocol Control Register */
	unsigned long ADC;	/* Address/Data Width Control */
};


void ixp2000_acquire_slowport(struct slowport_cfg *, struct slowport_cfg *);
void ixp2000_release_slowport(struct slowport_cfg *);

static inline unsigned ixp2000_has_broken_slowport(void)
{
	unsigned long id = *IXP2000_PRODUCT_ID;
	unsigned long id_prod = id & (IXP2000_MAJ_PROD_TYPE_MASK |
				      IXP2000_MIN_PROD_TYPE_MASK);
	return (((id_prod ==
		  /* fixed in IXP2400-B0 */
		  (IXP2000_MAJ_PROD_TYPE_IXP2000 |
		   IXP2000_MIN_PROD_TYPE_IXP2400)) &&
		 ((id & IXP2000_MAJ_REV_MASK) == 0)) ||
		((id_prod ==
		  /* fixed in IXP2800-B0 */
		  (IXP2000_MAJ_PROD_TYPE_IXP2000 |
		   IXP2000_MIN_PROD_TYPE_IXP2800)) &&
		 ((id & IXP2000_MAJ_REV_MASK) == 0)) ||
		((id_prod ==
		  /* fixed in IXP2850-B0 */
		  (IXP2000_MAJ_PROD_TYPE_IXP2000 |
		   IXP2000_MIN_PROD_TYPE_IXP2850)) &&
		 ((id & IXP2000_MAJ_REV_MASK) == 0)));
}

static inline unsigned int ixp2000_has_flash(void)
{
	return ((*IXP2000_STRAP_OPTIONS) & (CFG_BOOT_PROM));
}

static inline unsigned int ixp2000_is_pcimaster(void)
{
	return ((*IXP2000_STRAP_OPTIONS) & (CFG_PCI_BOOT_HOST));
}

void ixp2000_map_io(void);
void ixp2000_uart_init(void);
void ixp2000_init_irq(void);
void ixp2000_init_time(unsigned long);
unsigned long ixp2000_gettimeoffset(void);

struct pci_sys_data;

u32 *ixp2000_pci_config_addr(unsigned int bus, unsigned int devfn, int where);
void ixp2000_pci_preinit(void);
int ixp2000_pci_setup(int, struct pci_sys_data*);
struct pci_bus* ixp2000_pci_scan_bus(int, struct pci_sys_data*);
int ixp2000_pci_read_config(struct pci_bus*, unsigned int, int, int, u32 *);
int ixp2000_pci_write_config(struct pci_bus*, unsigned int, int, int, u32);

struct ixp2000_flash_data {
	struct flash_platform_data *platform_data;
	int nr_banks;
	unsigned long (*bank_setup)(unsigned long);
};

struct ixp2000_i2c_pins {
	unsigned long sda_pin;
	unsigned long scl_pin;
};


#endif /*  !__ASSEMBLY__ */
