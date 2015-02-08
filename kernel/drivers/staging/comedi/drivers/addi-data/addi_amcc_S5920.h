

#define AMCC_OP_REG_MCSR	0x3c
#define EEPROM_BUSY		0x80000000
#define NVCMD_LOAD_LOW		(0x4 << 5)	/* nvRam load low command */
#define NVCMD_LOAD_HIGH		(0x5 << 5)	/* nvRam load high command */
#define NVCMD_BEGIN_READ	(0x7 << 5)	/* nvRam begin read command */
#define NVCMD_BEGIN_WRITE	(0x6 << 5)	/* EEPROM begin write command */

int i_AddiHeaderRW_ReadEeprom(int i_NbOfWordsToRead,
			      unsigned int dw_PCIBoardEepromAddress,
			      unsigned short w_EepromStartAddress, unsigned short *pw_DataRead);
