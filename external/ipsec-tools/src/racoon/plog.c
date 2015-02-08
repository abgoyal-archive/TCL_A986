/*	$NetBSD: plog.c,v 1.4.6.2 2009/04/20 13:35:36 tteras Exp $	*/

/* Id: plog.c,v 1.11 2006/06/20 09:57:31 vanhu Exp */


#include "config.h"

#include <sys/types.h>
#include <sys/param.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#ifdef HAVE_STDARG_H
#include <stdarg.h>
#else
#include <varargs.h>
#endif
#if TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# if HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif
#include <ctype.h>
#include <err.h>

#include "var.h"
#include "misc.h"
#include "plog.h"
#include "logger.h"
#include "debug.h"
#include "gcmalloc.h"

#ifndef VA_COPY
# define VA_COPY(dst,src) memcpy(&(dst), &(src), sizeof(va_list))
#endif

char *pname = NULL;
u_int32_t loglevel = LLV_BASE;
int f_foreground = 0;

int print_location = 0;

static struct log *logp = NULL;
static char *logfile = NULL;

static char *plog_common __P((int, const char *, const char *));

static struct plogtags {
	char *name;
	int priority;
} ptab[] = {
	{ "(not defined)",	0, },
	{ "ERROR",		LOG_INFO, },
	{ "WARNING",		LOG_INFO, },
	{ "NOTIFY",		LOG_INFO, },
	{ "INFO",		LOG_INFO, },
	{ "DEBUG",		LOG_DEBUG, },
	{ "DEBUG2",		LOG_DEBUG, },
};

static char *
plog_common(pri, fmt, func)
	int pri;
	const char *fmt, *func;
{
	static char buf[800];	/* XXX shoule be allocated every time ? */
	char *p;
	int reslen, len;

	p = buf;
	reslen = sizeof(buf);

	if (logfile || f_foreground) {
		time_t t;
		struct tm *tm;

		t = time(0);
		tm = localtime(&t);
		len = strftime(p, reslen, "%Y-%m-%d %T: ", tm);
		p += len;
		reslen -= len;
	}

	if (pri < ARRAYLEN(ptab)) {
		len = snprintf(p, reslen, "%s: ", ptab[pri].name);
		if (len >= 0 && len < reslen) {
			p += len;
			reslen -= len;
		} else
			*p = '\0';
	}

	if (print_location)
		snprintf(p, reslen, "%s: %s", func, fmt);
	else
		snprintf(p, reslen, "%s", fmt);
#ifdef BROKEN_PRINTF
	while ((p = strstr(buf,"%z")) != NULL)
		p[1] = 'l';
#endif

	return buf;
}

void
_plog(int pri, const char *func, struct sockaddr *sa, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	plogv(pri, func, sa, fmt, ap);
	va_end(ap);
}

void
plogv(int pri, const char *func, struct sockaddr *sa,
	const char *fmt, va_list ap)
{
	char *newfmt;
	va_list ap_bak;

	if (pri > loglevel)
		return;

	newfmt = plog_common(pri, fmt, func);

	VA_COPY(ap_bak, ap);
	
	if (f_foreground)
		vprintf(newfmt, ap);

	if (logfile)
		log_vaprint(logp, newfmt, ap_bak);
	else {
		if (pri < ARRAYLEN(ptab))
			vsyslog(ptab[pri].priority, newfmt, ap_bak);
		else
			vsyslog(LOG_ALERT, newfmt, ap_bak);
	}
}

void
plogdump(pri, data, len)
	int pri;
	void *data;
	size_t len;
{
	caddr_t buf;
	size_t buflen;
	int i, j;

	if (pri > loglevel)
		return;

	/*
	 * 2 words a bytes + 1 space 4 bytes + 1 newline 32 bytes
	 * + 2 newline + '\0'
	 */
	buflen = (len * 2) + (len / 4) + (len / 32) + 3;
	buf = racoon_malloc(buflen);

	i = 0;
	j = 0;
	while (j < len) {
		if (j % 32 == 0)
			buf[i++] = '\n';
		else
		if (j % 4 == 0)
			buf[i++] = ' ';
		snprintf(&buf[i], buflen - i, "%02x",
			((unsigned char *)data)[j] & 0xff);
		i += 2;
		j++;
	}
	if (buflen - i >= 2) {
		buf[i++] = '\n';
		buf[i] = '\0';
	}
	plog(pri, LOCATION, NULL, "%s", buf);

	racoon_free(buf);
}

void
ploginit()
{
	if (logfile) {
		logp = log_open(250, logfile);
		if (logp == NULL)
			errx(1, "ERROR: failed to open log file %s.", logfile);
		return;
	}

        openlog(pname, LOG_NDELAY, LOG_DAEMON);
}

void
plogset(file)
	char *file;
{
	if (logfile != NULL)
		racoon_free(logfile);
	logfile = racoon_strdup(file);
	STRDUP_FATAL(logfile);
}

char*
binsanitize(binstr, n)
	char *binstr;
	size_t n;
{
	int p,q;
	char* d;

	d = racoon_malloc(n + 1);
	for (p = 0, q = 0; p < n; p++) {
		if (isgraph((int)binstr[p])) {
			d[q++] = binstr[p];
		} else {
			if (q && d[q - 1] != ' ')
				d[q++] = ' ';
		}
	}
	d[q++] = '\0';

	return d;
}
	