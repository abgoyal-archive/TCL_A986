/*	$NetBSD: getpass.c,v 1.15 2003/08/07 16:42:50 agc Exp $	*/


#if 0
#include <sys/cdefs.h>
#if defined(LIBC_SCCS) && !defined(lint)
#if 0
static char sccsid[] = "@(#)getpass.c	8.1 (Berkeley) 6/4/93";
#else
__RCSID("$NetBSD: getpass.c,v 1.15 2003/08/07 16:42:50 agc Exp $");
#endif
#endif /* LIBC_SCCS and not lint */

#include "namespace.h"
#endif

#include <assert.h>
#include <paths.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#if 0
#ifdef __weak_alias
__weak_alias(getpass,_getpass)
#endif
#endif

char *
getpass(prompt)
	const char *prompt;
{
	struct termios term;
	int ch;
	char *p;
	FILE *fp, *outfp;
	int echo;
	static char buf[_PASSWORD_LEN + 1];
	sigset_t oset, nset;

#if 0
	_DIAGASSERT(prompt != NULL);
#endif

	/*
	 * read and write to /dev/tty if possible; else read from
	 * stdin and write to stderr.
	 */
	if ((outfp = fp = fopen(_PATH_TTY, "w+")) == NULL) {
		outfp = stderr;
		fp = stdin;
	}

	/*
	 * note - blocking signals isn't necessarily the
	 * right thing, but we leave it for now.
	 */
	sigemptyset(&nset);
	sigaddset(&nset, SIGINT);
	sigaddset(&nset, SIGTSTP);
	(void)sigprocmask(SIG_BLOCK, &nset, &oset);

	(void)tcgetattr(fileno(fp), &term);
	if ((echo = (term.c_lflag & ECHO)) != 0) {
		term.c_lflag &= ~ECHO;
		(void)tcsetattr(fileno(fp), TCSAFLUSH /*|TCSASOFT*/, &term);
	}
	if (prompt != NULL)
		(void)fputs(prompt, outfp);
	rewind(outfp);			/* implied flush */
	for (p = buf; (ch = getc(fp)) != EOF && ch != '\n';)
		if (p < buf + _PASSWORD_LEN)
			*p++ = ch;
	*p = '\0';
	(void)write(fileno(outfp), "\n", 1);
	if (echo) {
		term.c_lflag |= ECHO;
		(void)tcsetattr(fileno(fp), TCSAFLUSH/*|TCSASOFT*/, &term);
	}
	(void)sigprocmask(SIG_SETMASK, &oset, NULL);
	if (fp != stdin)
		(void)fclose(fp);
	return(buf);
}
