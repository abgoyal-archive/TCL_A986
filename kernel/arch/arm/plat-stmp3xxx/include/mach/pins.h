

#ifndef __ASM_PLAT_PINS_H
#define __ASM_PLAT_PINS_H

#define STMP3XXX_PINID(bank, pin)	(bank * 32 + pin)
#define STMP3XXX_PINID_TO_BANK(pinid)	(pinid / 32)
#define STMP3XXX_PINID_TO_PINNUM(pinid)	(pinid % 32)

#define PINID_NO_PIN	STMP3XXX_PINID(0xFF, 0xFF)

#endif /* __ASM_PLAT_PINS_H */
