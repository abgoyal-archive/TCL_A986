

#ifndef _ASM_IA64_SN_TIO_CA_AGP_PROVIDER_H
#define _ASM_IA64_SN_TIO_CA_AGP_PROVIDER_H

#include <asm/sn/tioca.h>


#define TIOCA_WAR_ENABLED(pv, tioca_common) \
	((1 << tioca_common->ca_rev) & pv)

  /* TIO:ICE:FRZ:Freezer loses a PIO data ucred on PIO RD RSP with CW error */
#define PV907908 (1 << 1)
  /* ATI config space problems after BIOS execution starts */
#define PV908234 (1 << 1)
  /* CA:AGPDMA write request data mismatch with ABC1CL merge */
#define PV895469 (1 << 1)
  /* TIO:CA TLB invalidate of written GART entries possibly not occurring in CA*/
#define PV910244 (1 << 1)

struct tioca_dmamap{
	struct list_head	cad_list;	/* headed by ca_list */

	dma_addr_t		cad_dma_addr;	/* Linux dma handle */
	uint			cad_gart_entry; /* start entry in ca_gart_pagemap */
	uint			cad_gart_size;	/* #entries for this map */
};


struct tioca_common ;

struct tioca_kernel {
	struct tioca_common	*ca_common;	/* tioca this belongs to */
	struct list_head	ca_list;	/* list of all ca's */
	struct list_head	ca_dmamaps;
	spinlock_t		ca_lock;	/* Kernel lock */
	cnodeid_t		ca_closest_node;
	struct list_head	*ca_devices;	/* bus->devices */

	/*
	 * General GART stuff
	 */
	u64	ca_ap_size;		/* size of aperature in bytes */
	u32	ca_gart_entries;	/* # u64 entries in gart */
	u32	ca_ap_pagesize; 	/* aperature page size in bytes */
	u64	ca_ap_bus_base; 	/* bus address of CA aperature */
	u64	ca_gart_size;		/* gart size in bytes */
	u64	*ca_gart;		/* gart table vaddr */
	u64	ca_gart_coretalk_addr;	/* gart coretalk addr */
	u8		ca_gart_iscoherent;	/* used in tioca_tlbflush */

	/* PCI GART convenience values */
	u64	ca_pciap_base;		/* pci aperature bus base address */
	u64	ca_pciap_size;		/* pci aperature size (bytes) */
	u64	ca_pcigart_base;	/* gfx GART bus base address */
	u64	*ca_pcigart;		/* gfx GART vm address */
	u32	ca_pcigart_entries;
	u32	ca_pcigart_start;	/* PCI start index in ca_gart */
	void		*ca_pcigart_pagemap;

	/* AGP GART convenience values */
	u64	ca_gfxap_base;		/* gfx aperature bus base address */
	u64	ca_gfxap_size;		/* gfx aperature size (bytes) */
	u64	ca_gfxgart_base;	/* gfx GART bus base address */
	u64	*ca_gfxgart;		/* gfx GART vm address */
	u32	ca_gfxgart_entries;
	u32	ca_gfxgart_start;	/* agpgart start index in ca_gart */
};


struct tioca_common {
	struct pcibus_bussoft	ca_common;	/* common pciio header */

	u32		ca_rev;
	u32		ca_closest_nasid;

	u64		ca_prom_private;
	u64		ca_kernel_private;
};

static inline u64
tioca_paddr_to_gart(unsigned long paddr)
{
	/*
	 * We are assuming right now that paddr already has the correct
	 * format since the address from xtalk_dmaXXX should already have
	 * NODE_ID, CHIPLET_ID, and SYS_ADDR in the correct locations.
	 */

	return ((paddr) >> 12) | (1UL << 63);
}


static inline unsigned long
tioca_physpage_to_gart(u64 page_addr)
{
	u64 coretalk_addr;

	coretalk_addr = PHYS_TO_TIODMA(page_addr);
	if (!coretalk_addr) {
		return 0;
	}

	return tioca_paddr_to_gart(coretalk_addr);
}

static inline void
tioca_tlbflush(struct tioca_kernel *tioca_kernel)
{
	volatile u64 tmp;
	volatile struct tioca __iomem *ca_base;
	struct tioca_common *tioca_common;

	tioca_common = tioca_kernel->ca_common;
	ca_base = (struct tioca __iomem *)tioca_common->ca_common.bs_base;

	/*
	 * Explicit flushes not needed if GART is in cached mode
	 */
	if (tioca_kernel->ca_gart_iscoherent) {
		if (TIOCA_WAR_ENABLED(PV910244, tioca_common)) {
			/*
			 * PV910244:  RevA CA needs explicit flushes.
			 * Need to put GART into uncached mode before
			 * flushing otherwise the explicit flush is ignored.
			 *
			 * Alternate WAR would be to leave GART cached and
			 * touch every CL aligned GART entry.
			 */

			__sn_clrq_relaxed(&ca_base->ca_control2, CA_GART_MEM_PARAM);
			__sn_setq_relaxed(&ca_base->ca_control2, CA_GART_FLUSH_TLB);
			__sn_setq_relaxed(&ca_base->ca_control2,
			    (0x2ull << CA_GART_MEM_PARAM_SHFT));
			tmp = __sn_readq_relaxed(&ca_base->ca_control2);
		}

		return;
	}

	/*
	 * Gart in uncached mode ... need an explicit flush.
	 */

	__sn_setq_relaxed(&ca_base->ca_control2, CA_GART_FLUSH_TLB);
	tmp = __sn_readq_relaxed(&ca_base->ca_control2);
}

extern u32	tioca_gart_found;
extern struct list_head tioca_list;
extern int tioca_init_provider(void);
extern void tioca_fastwrite_enable(struct tioca_kernel *tioca_kern);
#endif /* _ASM_IA64_SN_TIO_CA_AGP_PROVIDER_H */
