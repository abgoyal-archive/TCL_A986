
/*	$NetBSD: print-ascii.c,v 1.1 1999/09/30 14:49:12 sjg Exp $ 	*/


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef lint
static const char rcsid[] _U_ =
     "@(#) $Header: /tcpdump/master/tcpdump/print-ascii.c,v 1.16.2.1 2005/07/06 20:54:49 guy Exp $";
#endif
#include <tcpdump-stdinc.h>
#include <stdio.h>

#include "interface.h"

#define ASCII_LINELENGTH 300
#define HEXDUMP_BYTES_PER_LINE 16
#define HEXDUMP_SHORTS_PER_LINE (HEXDUMP_BYTES_PER_LINE / 2)
#define HEXDUMP_HEXSTUFF_PER_SHORT 5 /* 4 hex digits and a space */
#define HEXDUMP_HEXSTUFF_PER_LINE \
		(HEXDUMP_HEXSTUFF_PER_SHORT * HEXDUMP_SHORTS_PER_LINE)

void
ascii_print(register const u_char *cp, register u_int length)
{
	register int s;

	putchar('\n');
	while (length > 0) {
		s = *cp++;
		length--;
		if (!isgraph(s) &&
		    (s != '\t' && s != ' ' && s != '\n' && s != '\r'))
			putchar('.');
		else
			putchar(s);
	}
}

void
hex_and_ascii_print_with_offset(register const char *ident,
    register const u_char *cp, register u_int length, register u_int oset)
{
	register u_int i;
	register int s1, s2;
	register int nshorts;
	char hexstuff[HEXDUMP_SHORTS_PER_LINE*HEXDUMP_HEXSTUFF_PER_SHORT+1], *hsp;
	char asciistuff[ASCII_LINELENGTH+1], *asp;

	nshorts = length / sizeof(u_short);
	i = 0;
	hsp = hexstuff; asp = asciistuff;
	while (--nshorts >= 0) {
		s1 = *cp++;
		s2 = *cp++;
		(void)snprintf(hsp, sizeof(hexstuff) - (hsp - hexstuff),
		    " %02x%02x", s1, s2);
		hsp += HEXDUMP_HEXSTUFF_PER_SHORT;
		*(asp++) = (isgraph(s1) ? s1 : '.');
		*(asp++) = (isgraph(s2) ? s2 : '.');
		i++;
		if (i >= HEXDUMP_SHORTS_PER_LINE) {
			*hsp = *asp = '\0';
			(void)printf("%s0x%04x: %-*s  %s",
			    ident, oset, HEXDUMP_HEXSTUFF_PER_LINE,
			    hexstuff, asciistuff);
			i = 0; hsp = hexstuff; asp = asciistuff;
			oset += HEXDUMP_BYTES_PER_LINE;
		}
	}
	if (length & 1) {
		s1 = *cp++;
		(void)snprintf(hsp, sizeof(hexstuff) - (hsp - hexstuff),
		    " %02x", s1);
		hsp += 3;
		*(asp++) = (isgraph(s1) ? s1 : '.');
		++i;
	}
	if (i > 0) {
		*hsp = *asp = '\0';
		(void)printf("%s0x%04x: %-*s  %s",
		     ident, oset, HEXDUMP_HEXSTUFF_PER_LINE,
		     hexstuff, asciistuff);
	}
}

void
hex_and_ascii_print(register const char *ident, register const u_char *cp,
    register u_int length)
{
	hex_and_ascii_print_with_offset(ident, cp, length, 0);
}

void
hex_print_with_offset(register const char *ident, register const u_char *cp, register u_int length,
		      register u_int oset)
{
	register u_int i, s;
	register int nshorts;

	nshorts = (u_int) length / sizeof(u_short);
	i = 0;
	while (--nshorts >= 0) {
		if ((i++ % 8) == 0) {
			(void)printf("%s0x%04x: ", ident, oset);
			oset += HEXDUMP_BYTES_PER_LINE;
		}
		s = *cp++;
		(void)printf(" %02x%02x", s, *cp++);
	}
	if (length & 1) {
		if ((i % 8) == 0)
			(void)printf("%s0x%04x: ", ident, oset);
		(void)printf(" %02x", *cp);
	}
}

void
hex_print(register const char *ident, register const u_char *cp, register u_int length)
{
	hex_print_with_offset(ident, cp, length, 0);
}

#ifdef MAIN
int
main(int argc, char *argv[])
{
	hex_print("\n\t", "Hello, World!\n", 14);
	printf("\n");
	hex_and_ascii_print("\n\t", "Hello, World!\n", 14);
	printf("\n");
	ascii_print("Hello, World!\n", 14);
	printf("\n");
#define TMSG "Now is the winter of our discontent...\n"
	hex_print_with_offset("\n\t", TMSG, sizeof(TMSG) - 1, 0x100);
	printf("\n");
	hex_and_ascii_print_with_offset("\n\t", TMSG, sizeof(TMSG) - 1, 0x100);
	printf("\n");
	exit(0);
}
#endif /* MAIN */
