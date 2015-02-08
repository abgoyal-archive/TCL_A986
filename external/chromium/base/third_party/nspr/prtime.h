/* Portions are Copyright (C) 2007 Google Inc */


#ifndef BASE_PRTIME_H__
#define BASE_PRTIME_H__

#include "base/logging.h"
#include "base/third_party/nspr/prtypes.h"

#define PR_ASSERT DCHECK

#define LL_I2L(l, i)    ((l) = (PRInt64)(i))
#define LL_MUL(r, a, b) ((r) = (a) * (b))

/**********************************************************************/
/************************* TYPES AND CONSTANTS ************************/
/**********************************************************************/

#define PR_MSEC_PER_SEC		1000UL
#define PR_USEC_PER_SEC		1000000UL
#define PR_NSEC_PER_SEC		1000000000UL
#define PR_USEC_PER_MSEC	1000UL
#define PR_NSEC_PER_MSEC	1000000UL


typedef PRInt64 PRTime;


typedef struct PRTimeParameters {
    PRInt32 tp_gmt_offset;     /* the offset from GMT in seconds */
    PRInt32 tp_dst_offset;     /* contribution of DST in seconds */
} PRTimeParameters;


typedef struct PRExplodedTime {
    PRInt32 tm_usec;		    /* microseconds past tm_sec (0-99999)  */
    PRInt32 tm_sec;             /* seconds past tm_min (0-61, accomodating
                                   up to two leap seconds) */	
    PRInt32 tm_min;             /* minutes past tm_hour (0-59) */
    PRInt32 tm_hour;            /* hours past tm_day (0-23) */
    PRInt32 tm_mday;            /* days past tm_mon (1-31, note that it
				                starts from 1) */
    PRInt32 tm_month;           /* months past tm_year (0-11, Jan = 0) */
    PRInt16 tm_year;            /* absolute year, AD (note that we do not
				                count from 1900) */

    PRInt8 tm_wday;		        /* calculated day of the week
				                (0-6, Sun = 0) */
    PRInt16 tm_yday;            /* calculated day of the year
				                (0-365, Jan 1 = 0) */

    PRTimeParameters tm_params;  /* time parameters used by conversion */
} PRExplodedTime;


typedef PRTimeParameters (PR_CALLBACK *PRTimeParamFn)(const PRExplodedTime *gmt);

/**********************************************************************/
/****************************** FUNCTIONS *****************************/
/**********************************************************************/

NSPR_API(PRTime)
PR_ImplodeTime(const PRExplodedTime *exploded);


NSPR_API(void) PR_NormalizeTime(
    PRExplodedTime *exploded, PRTimeParamFn params);

/**********************************************************************/
/*********************** TIME PARAMETER FUNCTIONS *********************/
/**********************************************************************/

/* Time parameters that represent Greenwich Mean Time */
NSPR_API(PRTimeParameters) PR_GMTParameters(const PRExplodedTime *gmt);


NSPR_API(PRStatus) PR_ParseTimeString (
	const char *string,
	PRBool default_to_gmt,
	PRTime *result);

#endif  // BASE_PRTIME_H__
