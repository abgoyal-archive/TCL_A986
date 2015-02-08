
/* to be used by qlogicfas and qlogic_cs */
#ifndef __QLOGICFAS408_H
#define __QLOGICFAS408_H

/*----------------------------------------------------------------*/
/* Configuration */


#define QL_TURBO_PDMA 1

/* This should be 1 to enable parity detection */

#define QL_ENABLE_PARITY 1


#define QL_RESET_AT_START 0


#define XTALFREQ	40

/**********/
/* DANGER! modify these at your own risk */
/**********/

/*****/
/* config register 1 (offset 8) options */
/* This needs to be set to 1 if your cabling is long or noisy */
#define SLOWCABLE 1

/*****/
/* offset 0xc */
#define FASTSCSI 0

/* This when set to 1 will set a faster sync transfer rate */
#define FASTCLK 0	/*(XTALFREQ>25?1:0)*/

/*****/
/* offset 6 */
#define SYNCXFRPD 5	/*(XTALFREQ/5)*/

/*****/
/* offset 7 */
#define SYNCOFFST 0

/*----------------------------------------------------------------*/

struct qlogicfas408_priv {
	int qbase;		/* Port */
	int qinitid;		/* initiator ID */
	int qabort;		/* Flag to cause an abort */
	int qlirq;		/* IRQ being used */
	int int_type;		/* type of irq, 2 for ISA board, 0 for PCMCIA */
	char qinfo[80];		/* description */
	struct scsi_cmnd *qlcmd;	/* current command being processed */
	struct Scsi_Host *shost;	/* pointer back to host */
	struct qlogicfas408_priv *next; /* next private struct */
};

/* The qlogic card uses two register maps - These macros select which one */
#define REG0 ( outb( inb( qbase + 0xd ) & 0x7f , qbase + 0xd ), outb( 4 , qbase + 0xd ))
#define REG1 ( outb( inb( qbase + 0xd ) | 0x80 , qbase + 0xd ), outb( 0xb4 | int_type, qbase + 0xd ))

/* following is watchdog timeout in microseconds */
#define WATCHDOG 5000000

/*----------------------------------------------------------------*/

#define rtrc(i) {}

#define get_priv_by_cmd(x) (struct qlogicfas408_priv *)&((x)->device->host->hostdata[0])
#define get_priv_by_host(x) (struct qlogicfas408_priv *)&((x)->hostdata[0])

irqreturn_t qlogicfas408_ihandl(int irq, void *dev_id);
int qlogicfas408_queuecommand(struct scsi_cmnd * cmd,
			      void (*done) (struct scsi_cmnd *));
int qlogicfas408_biosparam(struct scsi_device * disk,
			   struct block_device *dev,
			   sector_t capacity, int ip[]);
int qlogicfas408_abort(struct scsi_cmnd * cmd);
int qlogicfas408_bus_reset(struct scsi_cmnd * cmd);
const char *qlogicfas408_info(struct Scsi_Host *host);
int qlogicfas408_get_chip_type(int qbase, int int_type);
void qlogicfas408_setup(int qbase, int id, int int_type);
int qlogicfas408_detect(int qbase, int int_type);
void qlogicfas408_disable_ints(struct qlogicfas408_priv *priv);
#endif	/* __QLOGICFAS408_H */

