


#include <sys/cdefs.h>
#ifndef lint
__RCSID("$NetBSD: main.c,v 1.10 1997/10/01 02:18:14 enami Exp $");
static const char rcsid[] =
  "$FreeBSD: src/sbin/fsck_msdosfs/main.c,v 1.16 2009/06/10 19:02:54 avg Exp $";
#endif /* not lint */

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>

#include "fsutil.h"
#include "ext.h"

int alwaysno;		/* assume "no" for all questions */
int alwaysyes;		/* assume "yes" for all questions */
int preen;		/* set when preening */
int rdonly;		/* device is opened read only (supersedes above) */
int skipclean;		/* skip clean file systems if preening */
int debugmessage;	/* print verbose message for debugging */

static void usage(void);

static void
usage(void)
{

	fprintf(stderr, "%s\n%s\n",
	    "usage: fsck_msdosfs -p [-f] filesystem ...",
	    "       fsck_msdosfs [-ny] filesystem ...");
	exit(1);
}

int
main(int argc, char **argv)
{
	int ret = 0, erg;
	int ch;

	skipclean = 1;
	debugmessage = 0;
	while ((ch = getopt(argc, argv, "CfFnpy")) != -1) {
		switch (ch) {
		case 'C': /* for fsck_ffs compatibility */
			break;
		case 'f':
			skipclean = 0;
			break;
		case 'F':
			/*
			 * We can never run in the background.  We must exit
			 * silently with a nonzero exit code so that fsck(8)
			 * can probe our support for -F.  The exit code
			 * doesn't really matter, but we use an unusual one
			 * in case someone tries -F directly.  The -F flag
			 * is intentionally left out of the usage message.
			 */
			exit(5);
		case 'n':
			alwaysno = 1;
			alwaysyes = preen = 0;
			break;
		case 'y':
			alwaysyes = 1;
			alwaysno = preen = 0;
			break;

		case 'p':
			preen = 1;
			alwaysyes = alwaysno = 0;
			break;
		case 'd':
			debugmessage = 1;
			break;

		default:
			usage();
			break;
		}
	}
	argc -= optind;
	argv += optind;

	if (!argc)
		usage();

	while (--argc >= 0) {
//		setcdevname(*argv, preen);
		erg = checkfilesys(*argv++);
		if (erg > ret)
			ret = erg;
	}

	return ret;
}


/*VARARGS*/
int
ask(int def, const char *fmt, ...)
{
	va_list ap;

	char prompt[256];
	int c;

	if (preen) {
		if (rdonly)
			def = 0;
		if (def)
			printf("FIXED\n");
		return def;
	}

	va_start(ap, fmt);
	vsnprintf(prompt, sizeof(prompt), fmt, ap);
	va_end(ap);
	if (alwaysyes || rdonly) {
		printf("%s? %s\n", prompt, rdonly ? "no" : "yes");
		return !rdonly;
	}
	do {
		printf("%s? [yn] ", prompt);
		fflush(stdout);
		c = getchar();
		while (c != '\n' && getchar() != '\n')
			if (feof(stdin))
				return 0;
	} while (c != 'y' && c != 'Y' && c != 'n' && c != 'N');
	return (c == 'y' || c == 'Y');
}
