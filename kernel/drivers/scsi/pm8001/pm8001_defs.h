

#ifndef _PM8001_DEFS_H_
#define _PM8001_DEFS_H_

enum chip_flavors {
	chip_8001,
};
#define USI_MAX_MEMCNT			9
#define PM8001_MAX_DMA_SG		SG_ALL
enum phy_speed {
	PHY_SPEED_15 = 0x01,
	PHY_SPEED_30 = 0x02,
	PHY_SPEED_60 = 0x04,
};

enum data_direction {
	DATA_DIR_NONE = 0x0,	/* NO TRANSFER */
	DATA_DIR_IN = 0x01,	/* INBOUND */
	DATA_DIR_OUT = 0x02,	/* OUTBOUND */
	DATA_DIR_BYRECIPIENT = 0x04, /* UNSPECIFIED */
};

enum port_type {
	PORT_TYPE_SAS = (1L << 1),
	PORT_TYPE_SATA = (1L << 0),
};

/* driver compile-time configuration */
#define	PM8001_MAX_CCB		 512	/* max ccbs supported */
#define	PM8001_MAX_INB_NUM	 1
#define	PM8001_MAX_OUTB_NUM	 1
#define	PM8001_CAN_QUEUE	 128	/* SCSI Queue depth */

/* unchangeable hardware details */
#define	PM8001_MAX_PHYS		 8	/* max. possible phys */
#define	PM8001_MAX_PORTS	 8	/* max. possible ports */
#define	PM8001_MAX_DEVICES	 1024	/* max supported device */

enum memory_region_num {
	AAP1 = 0x0, /* application acceleration processor */
	IOP,	    /* IO processor */
	CI,	    /* consumer index */
	PI,	    /* producer index */
	IB,	    /* inbound queue */
	OB,	    /* outbound queue */
	NVMD,	    /* NVM device */
	DEV_MEM,    /* memory for devices */
	CCB_MEM,    /* memory for command control block */
};
#define	PM8001_EVENT_LOG_SIZE	 (128 * 1024)

/*error code*/
enum mpi_err {
	MPI_IO_STATUS_SUCCESS = 0x0,
	MPI_IO_STATUS_BUSY = 0x01,
	MPI_IO_STATUS_FAIL = 0x02,
};

enum phy_control_type {
	PHY_LINK_RESET = 0x01,
	PHY_HARD_RESET = 0x02,
	PHY_NOTIFY_ENABLE_SPINUP = 0x10,
};

enum pm8001_hba_info_flags {
	PM8001F_INIT_TIME	= (1U << 0),
	PM8001F_RUN_TIME	= (1U << 1),
};

#endif
