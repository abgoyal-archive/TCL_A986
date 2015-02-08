

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <media/cx25840.h>

#include "cx25821.h"
#include "tuner-xc2028.h"

/* board config info */

struct cx25821_board cx25821_boards[] = {
	[UNKNOWN_BOARD] = {
			   .name = "UNKNOWN/GENERIC",
			   /* Ensure safe default for unknown boards */
			   .clk_freq = 0,
			   },

	[CX25821_BOARD] = {
			   .name = "CX25821",
			   .portb = CX25821_RAW,
			   .portc = CX25821_264,
			   .input[0].type = CX25821_VMUX_COMPOSITE,
			   },

};

const unsigned int cx25821_bcount = ARRAY_SIZE(cx25821_boards);

struct cx25821_subid cx25821_subids[] = {
	{
	 .subvendor = 0x14f1,
	 .subdevice = 0x0920,
	 .card = CX25821_BOARD,
	 },
};

void cx25821_card_setup(struct cx25821_dev *dev)
{
	static u8 eeprom[256];

	if (dev->i2c_bus[0].i2c_rc == 0) {
		dev->i2c_bus[0].i2c_client.addr = 0xa0 >> 1;
		tveeprom_read(&dev->i2c_bus[0].i2c_client, eeprom,
			      sizeof(eeprom));
	}
}
