

#if HAVE_ERRNO_H
#include <errno.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#if HAVE_EXT2_IOCTLS
#include <sys/ioctl.h>
#endif

#include "e2p.h"

int getflags (int fd, unsigned long * flags)
{
	struct stat buf;
#if HAVE_STAT_FLAGS

	if (fstat (fd, &buf) == -1)
		return -1;

	*flags = 0;
#ifdef UF_IMMUTABLE
	if (buf.st_flags & UF_IMMUTABLE)
		*flags |= EXT2_IMMUTABLE_FL;
#endif
#ifdef UF_APPEND
	if (buf.st_flags & UF_APPEND)
		*flags |= EXT2_APPEND_FL;
#endif
#ifdef UF_NODUMP
	if (buf.st_flags & UF_NODUMP)
		*flags |= EXT2_NODUMP_FL;
#endif

	return 0;
#else
#if HAVE_EXT2_IOCTLS
	int r, f;
	
	if (!fstat(fd, &buf) &&
	    !S_ISREG(buf.st_mode) && !S_ISDIR(buf.st_mode))
		goto notsupp;
	r = ioctl (fd, EXT2_IOC_GETFLAGS, &f);
	*flags = f;
	return r;
#endif /* HAVE_EXT2_IOCTLS */
#endif
notsupp:
	errno = EOPNOTSUPP;
	return -1;
}
