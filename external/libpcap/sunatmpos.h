

/* SunATM header for ATM packet */
#define SUNATM_DIR_POS		0
#define SUNATM_VPI_POS		1
#define SUNATM_VCI_POS		2
#define SUNATM_PKT_BEGIN_POS	4	/* Start of ATM packet */

/* Protocol type values in the bottom for bits of the byte at SUNATM_DIR_POS. */
#define PT_LANE		0x01	/* LANE */
#define PT_LLC		0x02	/* LLC encapsulation */
#define PT_ILMI		0x05	/* ILMI */
#define PT_QSAAL	0x06	/* Q.SAAL */
