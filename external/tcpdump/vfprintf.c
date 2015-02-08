

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/vfprintf.c,v 1.6 2003/11/16 09:36:45 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>

#include "interface.h"

vfprintf(f, fmt, args)
	FILE *f;
	char *fmt;
	va_list args;
{
	int ret;

	if ((f->_flag & _IOWRT) == 0) {
		if (f->_flag & _IORW)
			f->_flag |= _IOWRT;
		else
			return EOF;
	}
	ret = _doprnt(fmt, args, f);
	return ferror(f) ? EOF : ret;
}
