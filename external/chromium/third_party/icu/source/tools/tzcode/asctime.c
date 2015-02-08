

#ifndef lint
#ifndef NOID
static char	elsieid[] = "@(#)asctime.c	8.2";
#endif /* !defined NOID */
#endif /* !defined lint */

/*LINTLIBRARY*/

#include "private.h"
#include "tzfile.h"

#ifdef __GNUC__
#define ASCTIME_FMT	"%.3s %.3s%3d %2.2d:%2.2d:%2.2d %-4s\n"
#else /* !defined __GNUC__ */
#define ASCTIME_FMT	"%.3s %.3s%3d %02.2d:%02.2d:%02.2d %-4s\n"
#endif /* !defined __GNUC__ */
#ifdef __GNUC__
#define ASCTIME_FMT_B	"%.3s %.3s%3d %2.2d:%2.2d:%2.2d     %s\n"
#else /* !defined __GNUC__ */
#define ASCTIME_FMT_B	"%.3s %.3s%3d %02.2d:%02.2d:%02.2d     %s\n"
#endif /* !defined __GNUC__ */

#define STD_ASCTIME_BUF_SIZE	26
#define MAX_ASCTIME_BUF_SIZE	(2*3+5*INT_STRLEN_MAXIMUM(int)+7+2+1+1)

static char	buf_asctime[MAX_ASCTIME_BUF_SIZE];


char *
asctime_r(timeptr, buf)
register const struct tm *	timeptr;
char *				buf;
{
	static const char	wday_name[][3] = {
		"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
	};
	static const char	mon_name[][3] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun",
		"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
	};
	register const char *	wn;
	register const char *	mn;
	char			year[INT_STRLEN_MAXIMUM(int) + 2];
	char			result[MAX_ASCTIME_BUF_SIZE];

	if (timeptr->tm_wday < 0 || timeptr->tm_wday >= DAYSPERWEEK)
		wn = "???";
	else	wn = wday_name[timeptr->tm_wday];
	if (timeptr->tm_mon < 0 || timeptr->tm_mon >= MONSPERYEAR)
		mn = "???";
	else	mn = mon_name[timeptr->tm_mon];
	/*
	** Use strftime's %Y to generate the year, to avoid overflow problems
	** when computing timeptr->tm_year + TM_YEAR_BASE.
	** Assume that strftime is unaffected by other out-of-range members
	** (e.g., timeptr->tm_mday) when processing "%Y".
	*/
	(void) strftime(year, sizeof year, "%Y", timeptr);
	/*
	** We avoid using snprintf since it's not available on all systems.
	*/
	(void) sprintf(result,
		((strlen(year) <= 4) ? ASCTIME_FMT : ASCTIME_FMT_B),
		wn, mn,
		timeptr->tm_mday, timeptr->tm_hour,
		timeptr->tm_min, timeptr->tm_sec,
		year);
	if (strlen(result) < STD_ASCTIME_BUF_SIZE || buf == buf_asctime) {
		(void) strcpy(buf, result);
		return buf;
	} else {
#ifdef EOVERFLOW
		errno = EOVERFLOW;
#else /* !defined EOVERFLOW */
		errno = EINVAL;
#endif /* !defined EOVERFLOW */
		return NULL;
	}
}


char *
asctime(timeptr)
register const struct tm *	timeptr;
{
	return asctime_r(timeptr, buf_asctime);
}
