
#include "defs.h"

const struct ioctlent ioctlent0[] = {
#include "ioctlent.h"
};

#ifdef LINUX
#include <asm/ioctl.h>
#endif

const int nioctlents0 = sizeof ioctlent0 / sizeof ioctlent0[0];

#if SUPPORTED_PERSONALITIES >= 2
const struct ioctlent ioctlent1[] = {
#include "ioctlent1.h"
};

const int nioctlents1 = sizeof ioctlent1 / sizeof ioctlent1[0];
#endif /* SUPPORTED_PERSONALITIES >= 2 */

#if SUPPORTED_PERSONALITIES >= 3
const struct ioctlent ioctlent2[] = {
#include "ioctlent2.h"
};

const int nioctlents2 = sizeof ioctlent2 / sizeof ioctlent2[0];
#endif /* SUPPORTED_PERSONALITIES >= 3 */

const struct ioctlent *ioctlent;
int nioctlents;

static int
compare(a, b)
const void *a;
const void *b;
{
	unsigned long code1 = ((struct ioctlent *) a)->code;
	unsigned long code2 = ((struct ioctlent *) b)->code;
	return (code1 > code2) ? 1 : (code1 < code2) ? -1 : 0;
}

const struct ioctlent *
ioctl_lookup(code)
long code;
{
	struct ioctlent *iop, ioent;

	ioent.code = code;
#ifdef LINUX
	ioent.code &= (_IOC_NRMASK<<_IOC_NRSHIFT) | (_IOC_TYPEMASK<<_IOC_TYPESHIFT);
#endif
	iop = (struct ioctlent *) bsearch((char *) &ioent, (char *) ioctlent,
			nioctlents, sizeof(struct ioctlent), compare);
	while (iop > ioctlent)
		if ((--iop)->code != ioent.code) {
			iop++;
			break;
		}
	return iop;
}

const struct ioctlent *
ioctl_next_match(iop)
const struct ioctlent *iop;
{
	long code;

	code = (iop++)->code;
	if (iop < ioctlent + nioctlents && iop->code == code)
		return iop;
	return NULL;
}

int
ioctl_decode(tcp, code, arg)
struct tcb *tcp;
long code, arg;
{
	switch ((code >> 8) & 0xff) {
#ifdef LINUX
#if defined(ALPHA) || defined(POWERPC)
	case 'f': case 't': case 'T':
#else /* !ALPHA */
	case 0x54:
#endif /* !ALPHA */
#else /* !LINUX */
	case 'f': case 't': case 'T':
#endif /* !LINUX */
		return term_ioctl(tcp, code, arg);
#ifdef LINUX
	case 0x89:
#else /* !LINUX */
	case 'r': case 's': case 'i':
#ifndef FREEBSD
	case 'p':
#endif
#endif /* !LINUX */
		return sock_ioctl(tcp, code, arg);
#ifdef USE_PROCFS
#ifndef HAVE_MP_PROCFS
#ifndef FREEBSD
	case 'q':
#else
	case 'p':
#endif
		return proc_ioctl(tcp, code, arg);
#endif
#endif /* USE_PROCFS */
#ifdef HAVE_SYS_STREAM_H
	case 'S':
		return stream_ioctl(tcp, code, arg);
#endif /* HAVE_SYS_STREAM_H */
#ifdef LINUX
	case 'p':
		return rtc_ioctl(tcp, code, arg);
#endif
	default:
		break;
	}
	return 0;
}

