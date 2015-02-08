
#ifndef _LINUX_CABLEMODEM_H_
#define _LINUX_CABLEMODEM_H_

/* some useful defines for sb1000.c e cmconfig.c - fv */
#define SIOCGCMSTATS		SIOCDEVPRIVATE+0	/* get cable modem stats */
#define SIOCGCMFIRMWARE		SIOCDEVPRIVATE+1	/* get cm firmware version */
#define SIOCGCMFREQUENCY	SIOCDEVPRIVATE+2	/* get cable modem frequency */
#define SIOCSCMFREQUENCY	SIOCDEVPRIVATE+3	/* set cable modem frequency */
#define SIOCGCMPIDS			SIOCDEVPRIVATE+4	/* get cable modem PIDs */
#define SIOCSCMPIDS			SIOCDEVPRIVATE+5	/* set cable modem PIDs */

#endif
