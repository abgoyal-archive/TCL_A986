

#ifndef __GRU_H__
#define __GRU_H__

#define GRU_CACHE_LINE_BYTES		64
#define GRU_HANDLE_STRIDE		256
#define GRU_CB_BASE			0
#define GRU_DS_BASE			0x20000

#if defined(CONFIG_IA64)
#define GRU_GSEG_PAGESIZE	(256 * 1024UL)
#elif defined(CONFIG_X86_64)
#define GRU_GSEG_PAGESIZE	(256 * 1024UL)		/* ZZZ 2MB ??? */
#else
#error "Unsupported architecture"
#endif

struct gru_chiplet_info {
	int	node;
	int	chiplet;
	int	blade;
	int	total_dsr_bytes;
	int	total_cbr;
	int	total_user_dsr_bytes;
	int	total_user_cbr;
	int	free_user_dsr_bytes;
	int	free_user_cbr;
};

struct gru_gseg_statistics {
	unsigned long	fmm_tlbmiss;
	unsigned long	upm_tlbmiss;
	unsigned long	tlbdropin;
	unsigned long	context_stolen;
	unsigned long	reserved[10];
};

/* Flags for GRU options on the gru_create_context() call */
/* Select one of the follow 4 options to specify how TLB misses are handled */
#define GRU_OPT_MISS_DEFAULT	0x0000	/* Use default mode */
#define GRU_OPT_MISS_USER_POLL	0x0001	/* User will poll CB for faults */
#define GRU_OPT_MISS_FMM_INTR	0x0002	/* Send interrupt to cpu to
					   handle fault */
#define GRU_OPT_MISS_FMM_POLL	0x0003	/* Use system polling thread */
#define GRU_OPT_MISS_MASK	0x0003	/* Mask for TLB MISS option */



#endif		/* __GRU_H__ */
