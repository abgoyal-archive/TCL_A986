
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include "../misc/nls-enable.h"

/* For Linux, define BLKFLSBUF if necessary */
#if (!defined(BLKFLSBUF) && defined(__linux__))
#define BLKFLSBUF	_IO(0x12,97)	/* flush buffer cache */
#endif

const char *progname;

static void usage(void)
{
	fprintf(stderr, _("Usage: %s disk\n"), progname);
	exit(1);
}	
	
int main(int argc, char **argv)
{
	int	fd;
	
	progname = argv[0];
	if (argc != 2)
		usage();

	fd = open(argv[1], O_RDONLY, 0);
	if (fd < 0) {
		perror("open");
		exit(1);
	}
	/*
	 * Note: to reread the partition table, use the ioctl
	 * BLKRRPART instead of BLKFSLBUF.
	 */
#ifdef BLKFLSBUF
	if (ioctl(fd, BLKFLSBUF, 0) < 0) {
		perror("ioctl BLKFLSBUF");
		exit(1);
	}
	return 0;
#else
	fprintf(stderr,
		_("BLKFLSBUF ioctl not supported!  Can't flush buffers.\n"));
	return 1;
#endif
}
