

#ifdef CONFIG_PPC64

#include <asm/prom.h>

#define atmel_getb(chip, offset) readb(chip->vendor->iobase + offset);
#define atmel_putb(val, chip, offset) writeb(val, chip->vendor->iobase + offset)
#define atmel_request_region request_mem_region
#define atmel_release_region release_mem_region

static inline void atmel_put_base_addr(void __iomem *iobase)
{
	iounmap(iobase);
}

static void __iomem * atmel_get_base_addr(unsigned long *base, int *region_size)
{
	struct device_node *dn;
	unsigned long address, size;
	const unsigned int *reg;
	int reglen;
	int naddrc;
	int nsizec;

	dn = of_find_node_by_name(NULL, "tpm");

	if (!dn)
		return NULL;

	if (!of_device_is_compatible(dn, "AT97SC3201")) {
		of_node_put(dn);
		return NULL;
	}

	reg = of_get_property(dn, "reg", &reglen);
	naddrc = of_n_addr_cells(dn);
	nsizec = of_n_size_cells(dn);

	of_node_put(dn);


	if (naddrc == 2)
		address = ((unsigned long) reg[0] << 32) | reg[1];
	else
		address = reg[0];

	if (nsizec == 2)
		size =
		    ((unsigned long) reg[naddrc] << 32) | reg[naddrc + 1];
	else
		size = reg[naddrc];

	*base = address;
	*region_size = size;
	return ioremap(*base, *region_size);
}
#else
#define atmel_getb(chip, offset) inb(chip->vendor->base + offset)
#define atmel_putb(val, chip, offset) outb(val, chip->vendor->base + offset)
#define atmel_request_region request_region
#define atmel_release_region release_region
/* Atmel definitions */
enum tpm_atmel_addr {
	TPM_ATMEL_BASE_ADDR_LO = 0x08,
	TPM_ATMEL_BASE_ADDR_HI = 0x09
};

/* Verify this is a 1.1 Atmel TPM */
static int atmel_verify_tpm11(void)
{

	/* verify that it is an Atmel part */
	if (tpm_read_index(TPM_ADDR, 4) != 'A' ||
	    tpm_read_index(TPM_ADDR, 5) != 'T' ||
	    tpm_read_index(TPM_ADDR, 6) != 'M' ||
	    tpm_read_index(TPM_ADDR, 7) != 'L')
		return 1;

	/* query chip for its version number */
	if (tpm_read_index(TPM_ADDR, 0x00) != 1 ||
	    tpm_read_index(TPM_ADDR, 0x01) != 1)
		return 1;

	/* This is an atmel supported part */
	return 0;
}

static inline void atmel_put_base_addr(void __iomem *iobase)
{
}

/* Determine where to talk to device */
static void __iomem * atmel_get_base_addr(unsigned long *base, int *region_size)
{
	int lo, hi;

	if (atmel_verify_tpm11() != 0)
		return NULL;

	lo = tpm_read_index(TPM_ADDR, TPM_ATMEL_BASE_ADDR_LO);
	hi = tpm_read_index(TPM_ADDR, TPM_ATMEL_BASE_ADDR_HI);

	*base = (hi << 8) | lo;
	*region_size = 2;

	return ioport_map(*base, *region_size);
}
#endif
