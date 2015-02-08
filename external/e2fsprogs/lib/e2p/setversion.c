

#if HAVE_ERRNO_H
#include <errno.h>
#endif
#include <sys/ioctl.h>

#include "e2p.h"

int setversion (int fd, unsigned long version)
{
#if HAVE_EXT2_IOCTLS
	int ver;

	ver = (int) version;
	return ioctl (fd, EXT2_IOC_SETVERSION, &ver);
#else /* ! HAVE_EXT2_IOCTLS */
	extern int errno;
	errno = EOPNOTSUPP;
	return -1;
#endif /* ! HAVE_EXT2_IOCTLS */
}
