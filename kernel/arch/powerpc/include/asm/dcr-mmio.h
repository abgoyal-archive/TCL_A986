

#ifndef _ASM_POWERPC_DCR_MMIO_H
#define _ASM_POWERPC_DCR_MMIO_H
#ifdef __KERNEL__

#include <asm/io.h>

typedef struct {
	void __iomem *token;
	unsigned int stride;
	unsigned int base;
} dcr_host_mmio_t;

static inline bool dcr_map_ok_mmio(dcr_host_mmio_t host)
{
	return host.token != NULL;
}

extern dcr_host_mmio_t dcr_map_mmio(struct device_node *dev,
				    unsigned int dcr_n,
				    unsigned int dcr_c);
extern void dcr_unmap_mmio(dcr_host_mmio_t host, unsigned int dcr_c);

static inline u32 dcr_read_mmio(dcr_host_mmio_t host, unsigned int dcr_n)
{
	return in_be32(host.token + ((host.base + dcr_n) * host.stride));
}

static inline void dcr_write_mmio(dcr_host_mmio_t host,
				  unsigned int dcr_n,
				  u32 value)
{
	out_be32(host.token + ((host.base + dcr_n) * host.stride), value);
}

extern u64 of_translate_dcr_address(struct device_node *dev,
				    unsigned int dcr_n,
				    unsigned int *stride);

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_DCR_MMIO_H */


