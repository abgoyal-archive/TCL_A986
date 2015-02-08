

#ifndef __BCMSDH_SDMMC_H__
#define __BCMSDH_SDMMC_H__

#define sd_err(x)
#define sd_trace(x)
#define sd_info(x)
#define sd_debug(x)
#define sd_data(x)
#define sd_ctrl(x)

#define sd_sync_dma(sd, read, nbytes)
#define sd_init_dma(sd)
#define sd_ack_intr(sd)
#define sd_wakeup(sd);

/* Allocate/init/free per-OS private data */
extern int sdioh_sdmmc_osinit(sdioh_info_t *sd);
extern void sdioh_sdmmc_osfree(sdioh_info_t *sd);

#define sd_log(x)

#define SDIOH_ASSERT(exp) \
	do { if (!(exp)) \
		printf("!!!ASSERT fail: file %s lines %d", __FILE__, __LINE__); \
	} while (0)

#define BLOCK_SIZE_4318 64
#define BLOCK_SIZE_4328 512

/* internal return code */
#define SUCCESS	0
#define ERROR	1

/* private bus modes */
#define SDIOH_MODE_SD4		2
#define CLIENT_INTR 		0x100	/* Get rid of this! */

struct sdioh_info {
	osl_t 		*osh;			/* osh handler */
	bool		client_intr_enabled;	/* interrupt connnected flag */
	bool		intr_handler_valid;	/* client driver interrupt handler valid */
	sdioh_cb_fn_t	intr_handler;		/* registered interrupt handler */
	void		*intr_handler_arg;	/* argument to call interrupt handler */
	uint16		intmask;		/* Current active interrupts */
	void		*sdos_info;		/* Pointer to per-OS private data */

	uint 		irq;			/* Client irq */
	int 		intrcount;		/* Client interrupts */

	bool		sd_use_dma;		/* DMA on CMD53 */
	bool 		sd_blockmode;		/* sd_blockmode == FALSE => 64 Byte Cmd 53s. */
						/*  Must be on for sd_multiblock to be effective */
	bool 		use_client_ints;	/* If this is false, make sure to restore */
	int 		sd_mode;		/* SD1/SD4/SPI */
	int 		client_block_size[SDIOD_MAX_IOFUNCS];		/* Blocksize */
	uint8 		num_funcs;		/* Supported funcs on client */
	uint32 		com_cis_ptr;
	uint32 		func_cis_ptr[SDIOD_MAX_IOFUNCS];
	uint		max_dma_len;
	uint		max_dma_descriptors;	/* DMA Descriptors supported by this controller. */
//	SDDMA_DESCRIPTOR	SGList[32];	/* Scatter/Gather DMA List */
};


/* Global message bits */
extern uint sd_msglevel;

/* OS-independent interrupt handler */
extern bool check_client_intr(sdioh_info_t *sd);

/* Core interrupt enable/disable of device interrupts */
extern void sdioh_sdmmc_devintr_on(sdioh_info_t *sd);
extern void sdioh_sdmmc_devintr_off(sdioh_info_t *sd);



/* Register mapping routines */
extern uint32 *sdioh_sdmmc_reg_map(osl_t *osh, int32 addr, int size);
extern void sdioh_sdmmc_reg_unmap(osl_t *osh, int32 addr, int size);

/* Interrupt (de)registration routines */
extern int sdioh_sdmmc_register_irq(sdioh_info_t *sd, uint irq);
extern void sdioh_sdmmc_free_irq(uint irq, sdioh_info_t *sd);

typedef struct _BCMSDH_SDMMC_INSTANCE {
	sdioh_info_t	*sd;
	struct sdio_func *func[SDIOD_MAX_IOFUNCS];
} BCMSDH_SDMMC_INSTANCE, *PBCMSDH_SDMMC_INSTANCE;

#endif /* __BCMSDH_SDMMC_H__ */
