


#ifndef _PPS_H_
#define _PPS_H_

#include <linux/types.h>

#define PPS_VERSION		"5.3.6"
#define PPS_MAX_SOURCES		16		/* should be enough... */



#define PPS_API_VERS_1		1
#define PPS_API_VERS		PPS_API_VERS_1	/* we use API version 1 */
#define PPS_MAX_NAME_LEN	32

struct pps_ktime {
	__s64 sec;
	__s32 nsec;
	__u32 flags;
};
#define PPS_TIME_INVALID	(1<<0)	/* used to specify timeout==NULL */

struct pps_kinfo {
	__u32 assert_sequence;		/* seq. num. of assert event */
	__u32 clear_sequence; 		/* seq. num. of clear event */
	struct pps_ktime assert_tu;	/* time of assert event */
	struct pps_ktime clear_tu;	/* time of clear event */
	int current_mode;		/* current mode bits */
};

struct pps_kparams {
	int api_version;		/* API version # */
	int mode;			/* mode bits */
	struct pps_ktime assert_off_tu;	/* offset compensation for assert */
	struct pps_ktime clear_off_tu;	/* offset compensation for clear */
};


/* Device/implementation parameters */
#define PPS_CAPTUREASSERT	0x01	/* capture assert events */
#define PPS_CAPTURECLEAR	0x02	/* capture clear events */
#define PPS_CAPTUREBOTH		0x03	/* capture assert and clear events */

#define PPS_OFFSETASSERT	0x10	/* apply compensation for assert ev. */
#define PPS_OFFSETCLEAR		0x20	/* apply compensation for clear ev. */

#define PPS_CANWAIT		0x100	/* can we wait for an event? */
#define PPS_CANPOLL		0x200	/* bit reserved for future use */

/* Kernel actions */
#define PPS_ECHOASSERT		0x40	/* feed back assert event to output */
#define PPS_ECHOCLEAR		0x80	/* feed back clear event to output */

/* Timestamp formats */
#define PPS_TSFMT_TSPEC		0x1000	/* select timespec format */
#define PPS_TSFMT_NTPFP		0x2000	/* select NTP format */


/* Kernel consumers */
#define PPS_KC_HARDPPS		0	/* hardpps() (or equivalent) */
#define PPS_KC_HARDPPS_PLL	1	/* hardpps() constrained to
					   use a phase-locked loop */
#define PPS_KC_HARDPPS_FLL	2	/* hardpps() constrained to
					   use a frequency-locked loop */

struct pps_fdata {
	struct pps_kinfo info;
	struct pps_ktime timeout;
};

#include <linux/ioctl.h>

#define PPS_GETPARAMS		_IOR('p', 0xa1, struct pps_kparams *)
#define PPS_SETPARAMS		_IOW('p', 0xa2, struct pps_kparams *)
#define PPS_GETCAP		_IOR('p', 0xa3, int *)
#define PPS_FETCH		_IOWR('p', 0xa4, struct pps_fdata *)

#endif /* _PPS_H_ */
