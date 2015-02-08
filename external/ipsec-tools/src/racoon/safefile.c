/*	$NetBSD: safefile.c,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/*	$KAME: safefile.c,v 1.5 2001/03/05 19:54:06 thorpej Exp $	*/


#include "config.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "plog.h"
#include "debug.h"
#include "misc.h"
#include "safefile.h"

int
safefile(path, secret)
	const char *path;
	int secret;
{
	struct stat s;
	uid_t me;

	/* no setuid */
	if (getuid() != geteuid()) {
		plog(LLV_ERROR, LOCATION, NULL,
		    "setuid'ed execution not allowed\n");
		return -1;
	}

	if (stat(path, &s) != 0)
		return -1;

	/* the file must be owned by the running uid */
	me = getuid();
	if (s.st_uid != me) {
		plog(LLV_ERROR, LOCATION, NULL,
		    "%s has invalid owner uid\n", path);
		return -1;
	}

	switch (s.st_mode & S_IFMT) {
	case S_IFREG:
		break;
	default:
		plog(LLV_ERROR, LOCATION, NULL,
		    "%s is an invalid file type 0x%x\n", path,
		    (s.st_mode & S_IFMT));
		return -1;
	}

	/* secret file should not be read by others */
	if (secret) {
		if ((s.st_mode & S_IRWXG) != 0 || (s.st_mode & S_IRWXO) != 0) {
			plog(LLV_ERROR, LOCATION, NULL,
			    "%s has weak file permission\n", path);
			return -1;
		}
	}

	return 0;
}
