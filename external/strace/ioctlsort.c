

#include <stdio.h>

#ifdef STDC_HEADERS
#include <stdlib.h>
#endif

struct ioctlent {
	char *doth;
	char *symbol;
	unsigned long code;
};

#include "ioctlent.raw"

int nioctlents = sizeof ioctlent / sizeof ioctlent[0];

int
compare(a, b)
const void *a;
const void *b;
{
	unsigned long code1 = ((struct ioctlent *) a)->code;
	unsigned long code2 = ((struct ioctlent *) b)->code;
	return (code1 > code2) ? 1 : (code1 < code2) ? -1 : 0;
}

int
main(argc, argv)
int argc;
char *argv[];
{
	int i;

	qsort(ioctlent, nioctlents, sizeof ioctlent[0], compare);
	for (i = 0; i < nioctlents; i++) {
		printf("{\"%s\", \"%s\", %#lx},\n",
			ioctlent[i].doth, ioctlent[i].symbol, ioctlent[i].code);
	}
	return 0;
}
