
#ifndef _TZ2ICU_H_
#define _TZ2ICU_H_


#define TZ_ICU_MAGIC "TZic" /* cf. TZ_MAGIC = "TZif" */

typedef unsigned char ICUZoneinfoVersion;

#define TZ_ICU_VERSION ((ICUZoneinfoVersion) 1)

#define ICU_ZONE_FILE "icu_zone.txt"

/* File containing legacy aliases.  Read by tz2icu. */
#define ICU_TZ_ALIAS  "tz.alias"

#define ICU_TZ_RESOURCE "zoneinfo"

#endif
