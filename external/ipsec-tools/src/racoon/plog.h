/*	$NetBSD: plog.h,v 1.4.6.1 2007/11/06 16:41:27 vanhu Exp $	*/

/* Id: plog.h,v 1.7 2006/06/20 09:57:31 vanhu Exp */


#ifndef _PLOG_H
#define _PLOG_H

#ifdef ANDROID_PATCHED

#define LLV_ERROR   0
#define LLV_WARNING 1
#define LLV_NOTIFY  2
#define LLV_INFO    3
#define LLV_DEBUG   4
#define LLV_DEBUG2  5

#define loglevel LLV_INFO

#define plog(level, location, address, ...)                 \
    do {                                                    \
        if ((level) >= LLV_ERROR && (level) <= LLV_INFO) {  \
            do_plog((level), __VA_ARGS__);                  \
        }                                                   \
    } while (0)

#define plogdump(...)

extern void do_plog(int level, char *format, ...);
extern char* binsanitize(char *binary, size_t size);

#else

#ifdef HAVE_STDARG_H
#include <stdarg.h>
#else
#include <varargs.h>
#endif
#include <syslog.h>

#define LLV_ERROR	1
#define LLV_WARNING	2
#define LLV_NOTIFY	3
#define LLV_INFO	4
#define LLV_DEBUG	5
#define LLV_DEBUG2	6
 
#define LLV_BASE	LLV_INFO /* by default log less than this value. */

extern char *pname;
extern u_int32_t loglevel;
extern int f_foreground;
extern int print_location;

struct sockaddr;
#define plog(pri, ...) \
	do { \
		if ((pri) <= loglevel) \
			_plog((pri), __VA_ARGS__); \
	} while (0)
extern void _plog __P((int, const char *, struct sockaddr *, const char *, ...))
	__attribute__ ((__format__ (__printf__, 4, 5)));
extern void plogv __P((int, const char *, struct sockaddr *,
	const char *, va_list));
extern void plogdump __P((int, void *, size_t));
extern void ploginit __P((void));
extern void plogset __P((char *));

extern char* binsanitize __P((char*, size_t));

#endif

#endif /* _PLOG_H */
