/*	$NetBSD: str2val.c,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/*	$KAME: str2val.c,v 1.11 2001/08/16 14:37:29 itojun Exp $	*/


#include "config.h"

#include <sys/types.h>
#include <sys/param.h>
#include <ctype.h>

#include <stdlib.h>
#include <stdio.h>

#include "str2val.h"
#include "gcmalloc.h"

caddr_t
val2str(buf, mlen)
	const char *buf;
	size_t mlen;
{
	caddr_t new;
	size_t len = (mlen * 2) + mlen / 8 + 10;
	size_t i, j;

	if ((new = racoon_malloc(len)) == 0) return(0);

	for (i = 0, j = 0; i < mlen; i++) {
		snprintf(&new[j], len - j, "%02x", (u_char)buf[i]);
		j += 2;
		if (i % 8 == 7) {
			new[j++] = ' ';
			new[j] = '\0';
		}
	}
	new[j] = '\0';

	return(new);
}

char *
str2val(str, base, len)
	const char *str;
	int base;
	size_t *len;
{
	int f;
	size_t i;
	char *dst;
	char *rp;
	const char *p;
	char b[3];

	i = 0;
	for (p = str; *p != '\0'; p++) {
		if (isxdigit((int)*p))
			i++;
		else if (isspace((int)*p))
			;
		else
			return NULL;
	}
	if (i == 0 || (i % 2) != 0)
		return NULL;
	i /= 2;

	if ((dst = racoon_malloc(i)) == NULL)
		return NULL;

	i = 0;
	f = 0;
	for (rp = dst, p = str; *p != '\0'; p++) {
		if (isxdigit((int)*p)) {
			if (!f) {
				b[0] = *p;
				f = 1;
			} else {
				b[1] = *p;
				b[2] = '\0';
				*rp++ = (char)strtol(b, NULL, base);
				i++;
				f = 0;
			}
		}
	}

	*len = i;

	return(dst);
}
