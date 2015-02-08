
#include <stdlib.h>
#include "../qemu-common.h"
#include "../qemu-char.h"
#include "../console.h"
#include "msmouse.h"

#define MSMOUSE_LO6(n) ((n) & 0x3f)
#define MSMOUSE_HI2(n) (((n) & 0xc0) >> 6)

static void msmouse_event(void *opaque,
                          int dx, int dy, int dz, int buttons_state)
{
    CharDriverState *chr = (CharDriverState *)opaque;

    unsigned char bytes[4] = { 0x40, 0x00, 0x00, 0x00 };

    /* Movement deltas */
    bytes[0] |= (MSMOUSE_HI2(dy) << 2) | MSMOUSE_HI2(dx);
    bytes[1] |= MSMOUSE_LO6(dx);
    bytes[2] |= MSMOUSE_LO6(dy);

    /* Buttons */
    bytes[0] |= (buttons_state & 0x01 ? 0x20 : 0x00);
    bytes[0] |= (buttons_state & 0x02 ? 0x10 : 0x00);
    bytes[3] |= (buttons_state & 0x04 ? 0x20 : 0x00);

    /* We always send the packet of, so that we do not have to keep track
       of previous state of the middle button. This can potentially confuse
       some very old drivers for two button mice though. */
    qemu_chr_read(chr, bytes, 4);
}

static int msmouse_chr_write (struct CharDriverState *s, const uint8_t *buf, int len)
{
    /* Ignore writes to mouse port */
    return len;
}

static void msmouse_chr_close (struct CharDriverState *chr)
{
    qemu_free (chr);
}

CharDriverState *qemu_chr_open_msmouse(void)
{
    CharDriverState *chr;

    chr = qemu_mallocz(sizeof(CharDriverState));
    chr->chr_write = msmouse_chr_write;
    chr->chr_close = msmouse_chr_close;

    qemu_add_mouse_event_handler(msmouse_event, chr, 0, "QEMU Microsoft Mouse");

    return chr;
}
