

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/util.c,v 1.95.2.6 2006/02/08 01:40:09 hannes Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <sys/stat.h>

#include <errno.h>
#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#endif
#include <pcap.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "interface.h"

int
fn_print(register const u_char *s, register const u_char *ep)
{
	register int ret;
	register u_char c;

	ret = 1;			/* assume truncated */
	while (ep == NULL || s < ep) {
		c = *s++;
		if (c == '\0') {
			ret = 0;
			break;
		}
		if (!isascii(c)) {
			c = toascii(c);
			putchar('M');
			putchar('-');
		}
		if (!isprint(c)) {
			c ^= 0x40;	/* DEL to ?, others to alpha */
			putchar('^');
		}
		putchar(c);
	}
	return(ret);
}

int
fn_printn(register const u_char *s, register u_int n,
	  register const u_char *ep)
{
	register u_char c;

	while (n > 0 && (ep == NULL || s < ep)) {
		n--;
		c = *s++;
		if (!isascii(c)) {
			c = toascii(c);
			putchar('M');
			putchar('-');
		}
		if (!isprint(c)) {
			c ^= 0x40;	/* DEL to ?, others to alpha */
			putchar('^');
		}
		putchar(c);
	}
	return (n == 0) ? 0 : 1;
}

int
fn_printzp(register const u_char *s, register u_int n,
	  register const u_char *ep)
{
	register int ret;
	register u_char c;

	ret = 1;			/* assume truncated */
	while (n > 0 && (ep == NULL || s < ep)) {
		n--;
		c = *s++;
		if (c == '\0') {
			ret = 0;
			break;
		}
		if (!isascii(c)) {
			c = toascii(c);
			putchar('M');
			putchar('-');
		}
		if (!isprint(c)) {
			c ^= 0x40;	/* DEL to ?, others to alpha */
			putchar('^');
		}
		putchar(c);
	}
	return (n == 0) ? 0 : ret;
}

void
ts_print(register const struct timeval *tvp)
{
	register int s;
	struct tm *tm;
	time_t Time;
	static unsigned b_sec;
	static unsigned b_usec;

	switch (tflag) {

	case 0: /* Default */
		s = (tvp->tv_sec + thiszone) % 86400;
		(void)printf("%02d:%02d:%02d.%06u ",
			     s / 3600, (s % 3600) / 60, s % 60,
			     (unsigned)tvp->tv_usec);
		break;

	case 1: /* No time stamp */
		break;

	case 2: /* Unix timeval style */
		(void)printf("%u.%06u ",
			     (unsigned)tvp->tv_sec,
			     (unsigned)tvp->tv_usec);
		break;

	case 3: /* Microseconds since previous packet */
		if (b_sec == 0) {
			printf("000000 ");
		} else {
			int d_usec = tvp->tv_usec - b_usec;
			int d_sec = tvp->tv_sec - b_sec;

			while (d_usec < 0) {
				d_usec += 1000000;
				d_sec--;
			}
			if (d_sec)
				printf("%d. ", d_sec);
			printf("%06d ", d_usec);
		}
		b_sec = tvp->tv_sec;
		b_usec = tvp->tv_usec;
		break;

	case 4: /* Default + Date*/
		s = (tvp->tv_sec + thiszone) % 86400;
		Time = (tvp->tv_sec + thiszone) - s;
		tm = gmtime (&Time);
		if (!tm)
			printf("Date fail  ");
		else
			printf("%04d-%02d-%02d ",
				   tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday);
		printf("%02d:%02d:%02d.%06u ",
			   s / 3600, (s % 3600) / 60, s % 60, (unsigned)tvp->tv_usec);
		break;
	}
}

void
relts_print(int secs)
{
	static const char *lengths[] = {"y", "w", "d", "h", "m", "s"};
	static const int seconds[] = {31536000, 604800, 86400, 3600, 60, 1};
	const char **l = lengths;
	const int *s = seconds;

	if (secs == 0) {
		(void)printf("0s");
		return;
	}
	if (secs < 0) {
		(void)printf("-");
		secs = -secs;
	}
	while (secs > 0) {
		if (secs >= *s) {
			(void)printf("%d%s", secs / *s, *l);
			secs -= (secs / *s) * *s;
		}
		s++;
		l++;
	}
}


int
print_unknown_data(const u_char *cp,const char *ident,int len)
{
	if (len < 0) {
		printf("%sDissector error: print_unknown_data called with negative length",
		    ident);
		return(0);
	}
	if (snapend - cp < len)
		len = snapend - cp;
	if (len < 0) {
		printf("%sDissector error: print_unknown_data called with pointer past end of packet",
		    ident);
		return(0);
	}
        hex_print(ident,cp,len);
	return(1); /* everything is ok */
}

const char *
tok2strbuf(register const struct tok *lp, register const char *fmt,
	   register int v, char *buf, size_t bufsize)
{
	if (lp != NULL) {
		while (lp->s != NULL) {
			if (lp->v == v)
				return (lp->s);
			++lp;
		}
	}
	if (fmt == NULL)
		fmt = "#%d";

	(void)snprintf(buf, bufsize, fmt, v);
	return (const char *)buf;
}

const char *
tok2str(register const struct tok *lp, register const char *fmt,
	register int v)
{
	static char buf[4][128];
	static int idx = 0;
	char *ret;

	ret = buf[idx];
	idx = (idx+1) & 3;
	return tok2strbuf(lp, fmt, v, ret, sizeof(buf[0]));
}

char *
bittok2str(register const struct tok *lp, register const char *fmt,
	   register int v)
{
        static char buf[256]; /* our stringbuffer */
        int buflen=0;
        register int rotbit; /* this is the bit we rotate through all bitpositions */
        register int tokval;

	while (lp->s != NULL && lp != NULL) {
            tokval=lp->v;   /* load our first value */
            rotbit=1;
            while (rotbit != 0) {
                /*
                 * lets AND the rotating bit with our token value
                 * and see if we have got a match
                 */
		if (tokval == (v&rotbit)) {
                    /* ok we have found something */
                    buflen+=snprintf(buf+buflen, sizeof(buf)-buflen, "%s, ",lp->s);
                    break;
                }
                rotbit=rotbit<<1; /* no match - lets shift and try again */
            }
            lp++;
	}

        if (buflen != 0) { /* did we find anything */
            /* yep, set the the trailing zero 2 bytes before to eliminate the last comma & whitespace */
            buf[buflen-2] = '\0';
            return (buf);
        }
        else {
            /* bummer - lets print the "unknown" message as advised in the fmt string if we got one */
            if (fmt == NULL)
		fmt = "#%d";
            (void)snprintf(buf, sizeof(buf), fmt, v);
            return (buf);
        }
}

const char *
tok2strary_internal(register const char **lp, int n, register const char *fmt,
	register int v)
{
	static char buf[128];

	if (v >= 0 && v < n && lp[v] != NULL)
		return lp[v];
	if (fmt == NULL)
		fmt = "#%d";
	(void)snprintf(buf, sizeof(buf), fmt, v);
	return (buf);
}


int
mask2plen (u_int32_t mask)
{
	u_int32_t bitmasks[33] = {
		0x00000000,
		0x80000000, 0xc0000000, 0xe0000000, 0xf0000000,
		0xf8000000, 0xfc000000, 0xfe000000, 0xff000000,
		0xff800000, 0xffc00000, 0xffe00000, 0xfff00000,
		0xfff80000, 0xfffc0000, 0xfffe0000, 0xffff0000,
		0xffff8000, 0xffffc000, 0xffffe000, 0xfffff000,
		0xfffff800, 0xfffffc00, 0xfffffe00, 0xffffff00,
		0xffffff80, 0xffffffc0, 0xffffffe0, 0xfffffff0,
		0xfffffff8, 0xfffffffc, 0xfffffffe, 0xffffffff
	};
	int prefix_len = 32;

	/* let's see if we can transform the mask into a prefixlen */
	while (prefix_len >= 0) {
		if (bitmasks[prefix_len] == mask)
			break;
		prefix_len--;
	}
	return (prefix_len);
}

/* VARARGS */
void
error(const char *fmt, ...)
{
	va_list ap;

	(void)fprintf(stderr, "%s: ", program_name);
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	if (*fmt) {
		fmt += strlen(fmt);
		if (fmt[-1] != '\n')
			(void)fputc('\n', stderr);
	}
	exit(1);
	/* NOTREACHED */
}

/* VARARGS */
void
warning(const char *fmt, ...)
{
	va_list ap;

	(void)fprintf(stderr, "%s: WARNING: ", program_name);
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	if (*fmt) {
		fmt += strlen(fmt);
		if (fmt[-1] != '\n')
			(void)fputc('\n', stderr);
	}
}

char *
copy_argv(register char **argv)
{
	register char **p;
	register u_int len = 0;
	char *buf;
	char *src, *dst;

	p = argv;
	if (*p == 0)
		return 0;

	while (*p)
		len += strlen(*p++) + 1;

	buf = (char *)malloc(len);
	if (buf == NULL)
		error("copy_argv: malloc");

	p = argv;
	dst = buf;
	while ((src = *p++) != NULL) {
		while ((*dst++ = *src++) != '\0')
			;
		dst[-1] = ' ';
	}
	dst[-1] = '\0';

	return buf;
}

#ifndef O_BINARY
#define O_BINARY	0
#endif

char *
read_infile(char *fname)
{
	register int i, fd, cc;
	register char *cp;
	struct stat buf;

	fd = open(fname, O_RDONLY|O_BINARY);
	if (fd < 0)
		error("can't open %s: %s", fname, pcap_strerror(errno));

	if (fstat(fd, &buf) < 0)
		error("can't stat %s: %s", fname, pcap_strerror(errno));

	cp = malloc((u_int)buf.st_size + 1);
	if (cp == NULL)
		error("malloc(%d) for %s: %s", (u_int)buf.st_size + 1,
			fname, pcap_strerror(errno));
	cc = read(fd, cp, (u_int)buf.st_size);
	if (cc < 0)
		error("read %s: %s", fname, pcap_strerror(errno));
	if (cc != buf.st_size)
		error("short read %s (%d != %d)", fname, cc, (int)buf.st_size);

	close(fd);
	/* replace "# comment" with spaces */
	for (i = 0; i < cc; i++) {
		if (cp[i] == '#')
			while (i < cc && cp[i] != '\n')
				cp[i++] = ' ';
	}
	cp[cc] = '\0';
	return (cp);
}

void
safeputs(const char *s, int maxlen)
{
    int idx = 0;
	while (*s && idx < maxlen) {
		safeputchar(*s);
                idx++;
		s++;
	}
}

void
safeputchar(int c)
{
	unsigned char ch;

	ch = (unsigned char)(c & 0xff);
	if (ch < 0x80 && isprint(ch))
		printf("%c", ch);
	else
		printf("\\%03o", ch);
}
