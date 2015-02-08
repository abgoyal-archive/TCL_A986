


#ifdef __linux__
#include "aic7xxx_osm.h"
#include "aic7xxx_inline.h"
#include "aic7xxx_93cx6.h"
#else
#include <dev/aic7xxx/aic7xxx_osm.h>
#include <dev/aic7xxx/aic7xxx_inline.h>
#include <dev/aic7xxx/aic7xxx_93cx6.h>
#endif

struct seeprom_cmd {
  	uint8_t len;
 	uint8_t bits[11];
};

/* Short opcodes for the c46 */
static const struct seeprom_cmd seeprom_ewen = {9, {1, 0, 0, 1, 1, 0, 0, 0, 0}};
static const struct seeprom_cmd seeprom_ewds = {9, {1, 0, 0, 0, 0, 0, 0, 0, 0}};

/* Long opcodes for the C56/C66 */
static const struct seeprom_cmd seeprom_long_ewen = {11, {1, 0, 0, 1, 1, 0, 0, 0, 0}};
static const struct seeprom_cmd seeprom_long_ewds = {11, {1, 0, 0, 0, 0, 0, 0, 0, 0}};

/* Common opcodes */
static const struct seeprom_cmd seeprom_write = {3, {1, 0, 1}};
static const struct seeprom_cmd seeprom_read  = {3, {1, 1, 0}};

#define CLOCK_PULSE(sd, rdy)				\
	while ((SEEPROM_STATUS_INB(sd) & rdy) == 0) {	\
		;  /* Do nothing */			\
	}						\
	(void)SEEPROM_INB(sd);	/* Clear clock */

static void
send_seeprom_cmd(struct seeprom_descriptor *sd, const struct seeprom_cmd *cmd)
{
	uint8_t temp;
	int i = 0;

	/* Send chip select for one clock cycle. */
	temp = sd->sd_MS ^ sd->sd_CS;
	SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
	CLOCK_PULSE(sd, sd->sd_RDY);

	for (i = 0; i < cmd->len; i++) {
		if (cmd->bits[i] != 0)
			temp ^= sd->sd_DO;
		SEEPROM_OUTB(sd, temp);
		CLOCK_PULSE(sd, sd->sd_RDY);
		SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
		CLOCK_PULSE(sd, sd->sd_RDY);
		if (cmd->bits[i] != 0)
			temp ^= sd->sd_DO;
	}
}

static void
reset_seeprom(struct seeprom_descriptor *sd)
{
	uint8_t temp;

	temp = sd->sd_MS;
	SEEPROM_OUTB(sd, temp);
	CLOCK_PULSE(sd, sd->sd_RDY);
	SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
	CLOCK_PULSE(sd, sd->sd_RDY);
	SEEPROM_OUTB(sd, temp);
	CLOCK_PULSE(sd, sd->sd_RDY);
}

int
ahc_read_seeprom(struct seeprom_descriptor *sd, uint16_t *buf,
		 u_int start_addr, u_int count)
{
	int i = 0;
	u_int k = 0;
	uint16_t v;
	uint8_t temp;

	/*
	 * Read the requested registers of the seeprom.  The loop
	 * will range from 0 to count-1.
	 */
	for (k = start_addr; k < count + start_addr; k++) {
		/*
		 * Now we're ready to send the read command followed by the
		 * address of the 16-bit register we want to read.
		 */
		send_seeprom_cmd(sd, &seeprom_read);

		/* Send the 6 or 8 bit address (MSB first, LSB last). */
		temp = sd->sd_MS ^ sd->sd_CS;
		for (i = (sd->sd_chip - 1); i >= 0; i--) {
			if ((k & (1 << i)) != 0)
				temp ^= sd->sd_DO;
			SEEPROM_OUTB(sd, temp);
			CLOCK_PULSE(sd, sd->sd_RDY);
			SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
			CLOCK_PULSE(sd, sd->sd_RDY);
			if ((k & (1 << i)) != 0)
				temp ^= sd->sd_DO;
		}

		/*
		 * Now read the 16 bit register.  An initial 0 precedes the
		 * register contents which begins with bit 15 (MSB) and ends
		 * with bit 0 (LSB).  The initial 0 will be shifted off the
		 * top of our word as we let the loop run from 0 to 16.
		 */
		v = 0;
		for (i = 16; i >= 0; i--) {
			SEEPROM_OUTB(sd, temp);
			CLOCK_PULSE(sd, sd->sd_RDY);
			v <<= 1;
			if (SEEPROM_DATA_INB(sd) & sd->sd_DI)
				v |= 1;
			SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
			CLOCK_PULSE(sd, sd->sd_RDY);
		}

		buf[k - start_addr] = v;

		/* Reset the chip select for the next command cycle. */
		reset_seeprom(sd);
	}
#ifdef AHC_DUMP_EEPROM
	printf("\nSerial EEPROM:\n\t");
	for (k = 0; k < count; k = k + 1) {
		if (((k % 8) == 0) && (k != 0)) {
			printf ("\n\t");
		}
		printf (" 0x%x", buf[k]);
	}
	printf ("\n");
#endif
	return (1);
}

int
ahc_write_seeprom(struct seeprom_descriptor *sd, uint16_t *buf,
		  u_int start_addr, u_int count)
{
	const struct seeprom_cmd *ewen, *ewds;
	uint16_t v;
	uint8_t temp;
	int i, k;

	/* Place the chip into write-enable mode */
	if (sd->sd_chip == C46) {
		ewen = &seeprom_ewen;
		ewds = &seeprom_ewds;
	} else if (sd->sd_chip == C56_66) {
		ewen = &seeprom_long_ewen;
		ewds = &seeprom_long_ewds;
	} else {
		printf("ahc_write_seeprom: unsupported seeprom type %d\n",
		       sd->sd_chip);
		return (0);
	}

	send_seeprom_cmd(sd, ewen);
	reset_seeprom(sd);

	/* Write all requested data out to the seeprom. */
	temp = sd->sd_MS ^ sd->sd_CS;
	for (k = start_addr; k < count + start_addr; k++) {
		/* Send the write command */
		send_seeprom_cmd(sd, &seeprom_write);

		/* Send the 6 or 8 bit address (MSB first). */
		for (i = (sd->sd_chip - 1); i >= 0; i--) {
			if ((k & (1 << i)) != 0)
				temp ^= sd->sd_DO;
			SEEPROM_OUTB(sd, temp);
			CLOCK_PULSE(sd, sd->sd_RDY);
			SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
			CLOCK_PULSE(sd, sd->sd_RDY);
			if ((k & (1 << i)) != 0)
				temp ^= sd->sd_DO;
		}

		/* Write the 16 bit value, MSB first */
		v = buf[k - start_addr];
		for (i = 15; i >= 0; i--) {
			if ((v & (1 << i)) != 0)
				temp ^= sd->sd_DO;
			SEEPROM_OUTB(sd, temp);
			CLOCK_PULSE(sd, sd->sd_RDY);
			SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
			CLOCK_PULSE(sd, sd->sd_RDY);
			if ((v & (1 << i)) != 0)
				temp ^= sd->sd_DO;
		}

		/* Wait for the chip to complete the write */
		temp = sd->sd_MS;
		SEEPROM_OUTB(sd, temp);
		CLOCK_PULSE(sd, sd->sd_RDY);
		temp = sd->sd_MS ^ sd->sd_CS;
		do {
			SEEPROM_OUTB(sd, temp);
			CLOCK_PULSE(sd, sd->sd_RDY);
			SEEPROM_OUTB(sd, temp ^ sd->sd_CK);
			CLOCK_PULSE(sd, sd->sd_RDY);
		} while ((SEEPROM_DATA_INB(sd) & sd->sd_DI) == 0);

		reset_seeprom(sd);
	}

	/* Put the chip back into write-protect mode */
	send_seeprom_cmd(sd, ewds);
	reset_seeprom(sd);

	return (1);
}

int
ahc_verify_cksum(struct seeprom_config *sc)
{
	int i;
	int maxaddr;
	uint32_t checksum;
	uint16_t *scarray;

	maxaddr = (sizeof(*sc)/2) - 1;
	checksum = 0;
	scarray = (uint16_t *)sc;

	for (i = 0; i < maxaddr; i++)
		checksum = checksum + scarray[i];
	if (checksum == 0
	 || (checksum & 0xFFFF) != sc->checksum) {
		return (0);
	} else {
		return(1);
	}
}
