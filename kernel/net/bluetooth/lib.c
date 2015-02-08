

/* Bluetooth kernel library. */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <asm/errno.h>

#include <net/bluetooth/bluetooth.h>

void baswap(bdaddr_t *dst, bdaddr_t *src)
{
	unsigned char *d = (unsigned char *) dst;
	unsigned char *s = (unsigned char *) src;
	unsigned int i;

	for (i = 0; i < 6; i++)
		d[i] = s[5 - i];
}
EXPORT_SYMBOL(baswap);

char *batostr(bdaddr_t *ba)
{
	static char str[2][18];
	static int i = 1;

	i ^= 1;
	sprintf(str[i], "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
		ba->b[0], ba->b[1], ba->b[2],
		ba->b[3], ba->b[4], ba->b[5]);

	return str[i];
}
EXPORT_SYMBOL(batostr);

/* Bluetooth error codes to Unix errno mapping */
int bt_err(__u16 code)
{
	switch (code) {
	case 0:
		return 0;

	case 0x01:
		return EBADRQC;

	case 0x02:
		return ENOTCONN;

	case 0x03:
		return EIO;

	case 0x04:
		return EHOSTDOWN;

	case 0x05:
		return EACCES;

	case 0x06:
		return EBADE;

	case 0x07:
		return ENOMEM;

	case 0x08:
		return ETIMEDOUT;

	case 0x09:
		return EMLINK;

	case 0x0a:
		return EMLINK;

	case 0x0b:
		return EALREADY;

	case 0x0c:
		return EBUSY;

	case 0x0d:
	case 0x0e:
	case 0x0f:
		return ECONNREFUSED;

	case 0x10:
		return ETIMEDOUT;

	case 0x11:
	case 0x27:
	case 0x29:
	case 0x20:
		return EOPNOTSUPP;

	case 0x12:
		return EINVAL;

	case 0x13:
	case 0x14:
	case 0x15:
		return ECONNRESET;

	case 0x16:
		return ECONNABORTED;

	case 0x17:
		return ELOOP;

	case 0x18:
		return EACCES;

	case 0x1a:
		return EPROTONOSUPPORT;

	case 0x1b:
		return ECONNREFUSED;

	case 0x19:
	case 0x1e:
	case 0x23:
	case 0x24:
	case 0x25:
		return EPROTO;

	default:
		return ENOSYS;
	}
}
EXPORT_SYMBOL(bt_err);
