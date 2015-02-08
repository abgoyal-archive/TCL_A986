

#if defined(LIBC_SCCS) && !defined(lint)
static const char rcsid[] =
    "@(#) $Header: /tcpdump/master/tcpdump/missing/strdup.c,v 1.1 2001/01/20 07:26:08 guy Exp $ (LBL)";
#endif /* LIBC_SCCS and not lint */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "interface.h"

char *
strdup(str)
	const char *str;
{
	size_t len;
	char *copy;

	len = strlen(str) + 1;
	if ((copy = malloc(len)) == NULL)
		return (NULL);
	memcpy(copy, str, len);
	return (copy);
}
