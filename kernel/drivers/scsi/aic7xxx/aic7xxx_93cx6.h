
#ifndef _AIC7XXX_93CX6_H_
#define _AIC7XXX_93CX6_H_

typedef enum {
	C46 = 6,
	C56_66 = 8
} seeprom_chip_t;

struct seeprom_descriptor {
	struct ahc_softc *sd_ahc;
	u_int sd_control_offset;
	u_int sd_status_offset;
	u_int sd_dataout_offset;
	seeprom_chip_t sd_chip;
	uint16_t sd_MS;
	uint16_t sd_RDY;
	uint16_t sd_CS;
	uint16_t sd_CK;
	uint16_t sd_DO;
	uint16_t sd_DI;
};


#define	SEEPROM_INB(sd) \
	ahc_inb(sd->sd_ahc, sd->sd_control_offset)
#define	SEEPROM_OUTB(sd, value)					\
do {								\
	ahc_outb(sd->sd_ahc, sd->sd_control_offset, value);	\
	ahc_flush_device_writes(sd->sd_ahc);			\
} while(0)

#define	SEEPROM_STATUS_INB(sd) \
	ahc_inb(sd->sd_ahc, sd->sd_status_offset)
#define	SEEPROM_DATA_INB(sd) \
	ahc_inb(sd->sd_ahc, sd->sd_dataout_offset)

int ahc_read_seeprom(struct seeprom_descriptor *sd, uint16_t *buf,
		     u_int start_addr, u_int count);
int ahc_write_seeprom(struct seeprom_descriptor *sd, uint16_t *buf,
		      u_int start_addr, u_int count);
int ahc_verify_cksum(struct seeprom_config *sc);

#endif /* _AIC7XXX_93CX6_H_ */
