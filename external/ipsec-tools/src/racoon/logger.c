/*	$NetBSD: logger.c,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/*	$KAME: logger.c,v 1.9 2002/09/03 14:37:03 itojun Exp $	*/


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

#include "logger.h"
#include "var.h"
#include "gcmalloc.h"

struct log *
log_open(siz, fname)
	size_t siz;
	char *fname;
{
	struct log *p;

	p = (struct log *)racoon_malloc(sizeof(*p));
	if (p == NULL)
		return NULL;
	memset(p, 0, sizeof(*p));

	p->buf = (char **)racoon_malloc(sizeof(char *) * siz);
	if (p->buf == NULL) {
		racoon_free(p);
		return NULL;
	}
	memset(p->buf, 0, sizeof(char *) * siz);

	p->tbuf = (time_t *)racoon_malloc(sizeof(time_t *) * siz);
	if (p->tbuf == NULL) {
		racoon_free(p->buf);
		racoon_free(p);
		return NULL;
	}
	memset(p->tbuf, 0, sizeof(time_t *) * siz);

	p->siz = siz;
	if (fname)
		p->fname = racoon_strdup(fname);

	return p;
}

void
log_add(p, str)
	struct log *p;
	char *str;
{
	/* syslog if p->fname == NULL? */
	if (p->buf[p->head])
		racoon_free(p->buf[p->head]);
	p->buf[p->head] = racoon_strdup(str);
	p->tbuf[p->head] = time(NULL);
	p->head++;
	p->head %= p->siz;
}

int
log_print(p, str)
	struct log *p;
	char *str;
{
	FILE *fp;

	if (p->fname == NULL)
		return -1;	/*XXX syslog?*/
	fp = fopen(p->fname, "a");
	if (fp == NULL)
		return -1;
	fprintf(fp, "%s", str);
	fclose(fp);

	return 0;
}

int
log_vprint(struct log *p, const char *fmt, ...)
{
	va_list ap;

	FILE *fp;

	if (p->fname == NULL)
		return -1;	/*XXX syslog?*/
	fp = fopen(p->fname, "a");
	if (fp == NULL)
		return -1;
	va_start(ap, fmt);
	vfprintf(fp, fmt, ap);
	va_end(ap);

	fclose(fp);

	return 0;
}

int
log_vaprint(struct log *p, const char *fmt, va_list ap)
{
	FILE *fp;

	if (p->fname == NULL)
		return -1;	/*XXX syslog?*/
	fp = fopen(p->fname, "a");
	if (fp == NULL)
		return -1;
	vfprintf(fp, fmt, ap);
	fclose(fp);

	return 0;
}

int
log_close(p)
	struct log *p;
{
	FILE *fp;
	int i, j;
	char ts[256];
	struct tm *tm;

	if (p->fname == NULL)
		goto nowrite;
	fp = fopen(p->fname, "a");
	if (fp == NULL)
		goto nowrite;

	for (i = 0; i < p->siz; i++) {
		j = (p->head + i) % p->siz;
		if (p->buf[j]) {
			tm = localtime(&p->tbuf[j]);
			strftime(ts, sizeof(ts), "%B %d %T", tm);
			fprintf(fp, "%s: %s\n", ts, p->buf[j]);
			if (*(p->buf[j] + strlen(p->buf[j]) - 1) != '\n')
				fprintf(fp, "\n");
		}
	}
	fclose(fp);

nowrite:
	log_free(p);
	return 0;
}

void
log_free(p)
	struct log *p;
{
	int i;

	for (i = 0; i < p->siz; i++)
		racoon_free(p->buf[i]);
	racoon_free(p->buf);
	racoon_free(p->tbuf);
	if (p->fname)
		racoon_free(p->fname);
	racoon_free(p);
}

#ifdef TEST
struct log *l;

void
vatest(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	log_vaprint(l, fmt, ap);
	va_end(ap);
}

int
main(argc, argv)
	int argc;
	char **argv;
{
	int i;

	l = log_open(30, "/tmp/hoge");
	if (l == NULL)
		errx(1, "hoge");

	for (i = 0; i < 50; i++) {
		log_add(l, "foo");
		log_add(l, "baa");
		log_add(l, "baz");
	}
	log_print(l, "hoge\n");
	log_vprint(l, "hoge %s\n", "this is test");
	vatest("%s %s\n", "this is", "vprint test");
	abort();
	log_free(l);
}

#endif

