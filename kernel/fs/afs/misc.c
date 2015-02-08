

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <rxrpc/packet.h>
#include "internal.h"
#include "afs_fs.h"

int afs_abort_to_error(u32 abort_code)
{
	switch (abort_code) {
	case 13:		return -EACCES;
	case 27:		return -EFBIG;
	case 30:		return -EROFS;
	case VSALVAGE:		return -EIO;
	case VNOVNODE:		return -ENOENT;
	case VNOVOL:		return -ENOMEDIUM;
	case VVOLEXISTS:	return -EEXIST;
	case VNOSERVICE:	return -EIO;
	case VOFFLINE:		return -ENOENT;
	case VONLINE:		return -EEXIST;
	case VDISKFULL:		return -ENOSPC;
	case VOVERQUOTA:	return -EDQUOT;
	case VBUSY:		return -EBUSY;
	case VMOVED:		return -ENXIO;
	case 0x2f6df0a:		return -EWOULDBLOCK;
	case 0x2f6df0c:		return -EACCES;
	case 0x2f6df0f:		return -EBUSY;
	case 0x2f6df10:		return -EEXIST;
	case 0x2f6df11:		return -EXDEV;
	case 0x2f6df13:		return -ENOTDIR;
	case 0x2f6df14:		return -EISDIR;
	case 0x2f6df15:		return -EINVAL;
	case 0x2f6df1a:		return -EFBIG;
	case 0x2f6df1b:		return -ENOSPC;
	case 0x2f6df1d:		return -EROFS;
	case 0x2f6df1e:		return -EMLINK;
	case 0x2f6df20:		return -EDOM;
	case 0x2f6df21:		return -ERANGE;
	case 0x2f6df22:		return -EDEADLK;
	case 0x2f6df23:		return -ENAMETOOLONG;
	case 0x2f6df24:		return -ENOLCK;
	case 0x2f6df26:		return -ENOTEMPTY;
	case 0x2f6df78:		return -EDQUOT;

	case RXKADINCONSISTENCY: return -EPROTO;
	case RXKADPACKETSHORT:	return -EPROTO;
	case RXKADLEVELFAIL:	return -EKEYREJECTED;
	case RXKADTICKETLEN:	return -EKEYREJECTED;
	case RXKADOUTOFSEQUENCE: return -EPROTO;
	case RXKADNOAUTH:	return -EKEYREJECTED;
	case RXKADBADKEY:	return -EKEYREJECTED;
	case RXKADBADTICKET:	return -EKEYREJECTED;
	case RXKADUNKNOWNKEY:	return -EKEYREJECTED;
	case RXKADEXPIRED:	return -EKEYEXPIRED;
	case RXKADSEALEDINCON:	return -EKEYREJECTED;
	case RXKADDATALEN:	return -EKEYREJECTED;
	case RXKADILLEGALLEVEL:	return -EKEYREJECTED;

	default:		return -EREMOTEIO;
	}
}
