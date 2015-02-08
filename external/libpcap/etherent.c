

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/libpcap/etherent.c,v 1.22 2003/11/15 23:23:57 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>

#include <ctype.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>

#include "pcap-int.h"

#include <pcap-namedb.h>

#ifdef HAVE_OS_PROTO_H
#include "os-proto.h"
#endif

static inline int xdtoi(int);
static inline int skip_space(FILE *);
static inline int skip_line(FILE *);

/* Hex digit to integer. */
static inline int
xdtoi(c)
	register int c;
{
	if (isdigit(c))
		return c - '0';
	else if (islower(c))
		return c - 'a' + 10;
	else
		return c - 'A' + 10;
}

static inline int
skip_space(f)
	FILE *f;
{
	int c;

	do {
		c = getc(f);
	} while (isspace(c) && c != '\n');

	return c;
}

static inline int
skip_line(f)
	FILE *f;
{
	int c;

	do
		c = getc(f);
	while (c != '\n' && c != EOF);

	return c;
}

struct pcap_etherent *
pcap_next_etherent(FILE *fp)
{
	register int c, d, i;
	char *bp;
	static struct pcap_etherent e;

	memset((char *)&e, 0, sizeof(e));
	do {
		/* Find addr */
		c = skip_space(fp);
		if (c == '\n')
			continue;

		/* If this is a comment, or first thing on line
		   cannot be etehrnet address, skip the line. */
		if (!isxdigit(c)) {
			c = skip_line(fp);
			continue;
		}

		/* must be the start of an address */
		for (i = 0; i < 6; i += 1) {
			d = xdtoi(c);
			c = getc(fp);
			if (isxdigit(c)) {
				d <<= 4;
				d |= xdtoi(c);
				c = getc(fp);
			}
			e.addr[i] = d;
			if (c != ':')
				break;
			c = getc(fp);
		}
		if (c == EOF)
			break;

		/* Must be whitespace */
		if (!isspace(c)) {
			c = skip_line(fp);
			continue;
		}
		c = skip_space(fp);

		/* hit end of line... */
		if (c == '\n')
			continue;

		if (c == '#') {
			c = skip_line(fp);
			continue;
		}

		/* pick up name */
		bp = e.name;
		/* Use 'd' to prevent buffer overflow. */
		d = sizeof(e.name) - 1;
		do {
			*bp++ = c;
			c = getc(fp);
		} while (!isspace(c) && c != EOF && --d > 0);
		*bp = '\0';

		/* Eat trailing junk */
		if (c != '\n')
			(void)skip_line(fp);

		return &e;

	} while (c != EOF);

	return (NULL);
}
