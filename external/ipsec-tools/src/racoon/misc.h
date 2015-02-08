/*	$NetBSD: misc.h,v 1.4.6.1 2008/07/15 00:55:48 mgrooms Exp $	*/

/* Id: misc.h,v 1.9 2006/04/06 14:00:06 manubsd Exp */


#ifndef _MISC_H
#define _MISC_H

#define BIT2STR(b) bit2str(b, sizeof(b)<<3)

#ifdef HAVE_FUNC_MACRO
#define LOCATION        debug_location(__FILE__, __LINE__, __func__)
#else
#define LOCATION        debug_location(__FILE__, __LINE__, NULL)
#endif

extern int racoon_hexdump __P((void *, size_t));
extern char *bit2str __P((int, int));
extern void *get_newbuf __P((void *, size_t));
extern const char *debug_location __P((const char *, int, const char *));
extern int getfsize __P((char *));
struct timeval;
extern double timedelta __P((struct timeval *, struct timeval *));
char *strdup __P((const char *));

#if defined(__APPLE__) && defined(__MACH__)
#define RACOON_TAILQ_FOREACH_REVERSE(var, head, headname ,field)	\
  TAILQ_FOREACH_REVERSE(var, head, field, headname)
#else
#define RACOON_TAILQ_FOREACH_REVERSE(var, head, headname ,field)	\
    TAILQ_FOREACH_REVERSE(var, head, headname, field)
#endif

#ifndef HAVE_STRLCPY
#define strlcpy(d,s,l) (strncpy(d,s,l), (d)[(l)-1] = '\0')
#endif

#ifndef HAVE_STRLCAT
#define strlcat(d,s,l) strncat(d,s,(l)-strlen(d)-1)
#endif

#define STRDUP_FATAL(x) if (x == NULL) {			\
	plog(LLV_ERROR, LOCATION, NULL, "strdup failed\n");	\
	exit(1);						\
}

#include "libpfkey.h"

#endif /* _MISC_H */
