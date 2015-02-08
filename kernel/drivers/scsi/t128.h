


#ifndef T128_H
#define T128_H

#define T128_PUBLIC_RELEASE 3

#define TDEBUG		0
#define TDEBUG_INIT	0x1
#define TDEBUG_TRANSFER 0x2


#define T_ROM_OFFSET		0

#define T_RAM_OFFSET		0x1800

#define T_CONTROL_REG_OFFSET	0x1c00	/* rw */
#define T_CR_INT		0x10	/* Enable interrupts */
#define T_CR_CT			0x02	/* Reset watchdog timer */

#define T_STATUS_REG_OFFSET	0x1c20	/* ro */
#define T_ST_BOOT		0x80	/* Boot switch */
#define T_ST_S3			0x40	/* User settable switches, */
#define T_ST_S2			0x20	/* read 0 when switch is on, 1 off */
#define T_ST_S1			0x10
#define T_ST_PS2		0x08	/* Set for Microchannel 228 */
#define T_ST_RDY		0x04	/* 5380 DRQ */
#define T_ST_TIM		0x02	/* indicates 40us watchdog timer fired */
#define T_ST_ZERO		0x01	/* Always zero */

#define T_5380_OFFSET		0x1d00	/* 8 registers here, see NCR5380.h */

#define T_DATA_REG_OFFSET	0x1e00	/* rw 512 bytes long */

#ifndef ASM
static int t128_abort(struct scsi_cmnd *);
static int t128_biosparam(struct scsi_device *, struct block_device *,
			  sector_t, int*);
static int t128_detect(struct scsi_host_template *);
static int t128_queue_command(struct scsi_cmnd *,
			      void (*done)(struct scsi_cmnd *));
static int t128_bus_reset(struct scsi_cmnd *);

#ifndef CMD_PER_LUN
#define CMD_PER_LUN 2
#endif

#ifndef CAN_QUEUE
#define CAN_QUEUE 32
#endif

#ifndef HOSTS_C

#define NCR5380_implementation_fields \
    void __iomem *base

#define NCR5380_local_declare() \
    void __iomem *base

#define NCR5380_setup(instance) \
    base = ((struct NCR5380_hostdata *)(instance->hostdata))->base

#define T128_address(reg) (base + T_5380_OFFSET + ((reg) * 0x20))

#if !(TDEBUG & TDEBUG_TRANSFER)
#define NCR5380_read(reg) readb(T128_address(reg))
#define NCR5380_write(reg, value) writeb((value),(T128_address(reg)))
#else
#define NCR5380_read(reg)						\
    (((unsigned char) printk("scsi%d : read register %d at address %08x\n"\
    , instance->hostno, (reg), T128_address(reg))), readb(T128_address(reg)))

#define NCR5380_write(reg, value) {					\
    printk("scsi%d : write %02x to register %d at address %08x\n",	\
	    instance->hostno, (value), (reg), T128_address(reg));	\
    writeb((value), (T128_address(reg)));				\
}
#endif

#define NCR5380_intr t128_intr
#define do_NCR5380_intr do_t128_intr
#define NCR5380_queue_command t128_queue_command
#define NCR5380_abort t128_abort
#define NCR5380_bus_reset t128_bus_reset
#define NCR5380_proc_info t128_proc_info


#define T128_IRQS 0xc4a8

#endif /* else def HOSTS_C */
#endif /* ndef ASM */
#endif /* T128_H */
