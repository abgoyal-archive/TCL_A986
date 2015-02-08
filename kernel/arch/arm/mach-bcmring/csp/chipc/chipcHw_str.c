
/****************************************************************************/
/****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <mach/csp/chipcHw_inline.h>

/* ---- Private Constants and Types --------------------------------------- */

static const char *gMuxStr[] = {
	"GPIO",			/* 0 */
	"KeyPad",		/* 1 */
	"I2C-Host",		/* 2 */
	"SPI",			/* 3 */
	"Uart",			/* 4 */
	"LED-Mtx-P",		/* 5 */
	"LED-Mtx-S",		/* 6 */
	"SDIO-0",		/* 7 */
	"SDIO-1",		/* 8 */
	"PCM",			/* 9 */
	"I2S",			/* 10 */
	"ETM",			/* 11 */
	"Debug",		/* 12 */
	"Misc",			/* 13 */
	"0xE",			/* 14 */
	"0xF",			/* 15 */
};

/****************************************************************************/
/****************************************************************************/

const char *chipcHw_getGpioPinFunctionStr(int pin)
{
	if ((pin < 0) || (pin >= chipcHw_GPIO_COUNT)) {
		return "";
	}

	return gMuxStr[chipcHw_getGpioPinFunction(pin)];
}
