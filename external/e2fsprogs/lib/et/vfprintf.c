
#if defined(LIBC_SCCS) && !defined(lint)
static char sccsid[] = "@(#)vfprintf.c	5.2 (Berkeley) 6/27/88";
#endif /* LIBC_SCCS and not lint */

#if !HAVE_VPRINTF && HAVE_DOPRNT
#include <stdio.h>
#include <varargs.h>

int
vfprintf(iop, fmt, ap)
	FILE *iop;
	char *fmt;
	va_list ap;
{
	int len;
	char localbuf[BUFSIZ];

	if (iop->_flag & _IONBF) {
		iop->_flag &= ~_IONBF;
		iop->_ptr = iop->_base = localbuf;
		len = _doprnt(fmt, ap, iop);
		(void) fflush(iop);
		iop->_flag |= _IONBF;
		iop->_base = NULL;
		iop->_bufsiz = 0;
		iop->_cnt = 0;
	} else
		len = _doprnt(fmt, ap, iop);

	return (ferror(iop) ? EOF : len);
}
#endif /* !HAVE_VPRINTF */
