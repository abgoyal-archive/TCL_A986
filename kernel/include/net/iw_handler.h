

#ifndef _IW_HANDLER_H
#define _IW_HANDLER_H

/************************** DOCUMENTATION **************************/

/* ---------------------- THE IMPLEMENTATION ---------------------- */

/***************************** INCLUDES *****************************/

#include <linux/wireless.h>		/* IOCTL user space API */
#include <linux/if_ether.h>

/***************************** VERSION *****************************/
#define IW_HANDLER_VERSION	8


/**************************** CONSTANTS ****************************/

/* Enhanced spy support available */
#define IW_WIRELESS_SPY
#define IW_WIRELESS_THRSPY

#define EIWCOMMIT	EINPROGRESS

/* Flags available in struct iw_request_info */
#define IW_REQUEST_FLAG_COMPAT	0x0001	/* Compat ioctl call */

/* Type of headers we know about (basically union iwreq_data) */
#define IW_HEADER_TYPE_NULL	0	/* Not available */
#define IW_HEADER_TYPE_CHAR	2	/* char [IFNAMSIZ] */
#define IW_HEADER_TYPE_UINT	4	/* __u32 */
#define IW_HEADER_TYPE_FREQ	5	/* struct iw_freq */
#define IW_HEADER_TYPE_ADDR	6	/* struct sockaddr */
#define IW_HEADER_TYPE_POINT	8	/* struct iw_point */
#define IW_HEADER_TYPE_PARAM	9	/* struct iw_param */
#define IW_HEADER_TYPE_QUAL	10	/* struct iw_quality */

/* Handling flags */
#define IW_DESCR_FLAG_NONE	0x0000	/* Obvious */
/* Wrapper level flags */
#define IW_DESCR_FLAG_DUMP	0x0001	/* Not part of the dump command */
#define IW_DESCR_FLAG_EVENT	0x0002	/* Generate an event on SET */
#define IW_DESCR_FLAG_RESTRICT	0x0004	/* GET : request is ROOT only */
				/* SET : Omit payload from generated iwevent */
#define IW_DESCR_FLAG_NOMAX	0x0008	/* GET : no limit on request size */
/* Driver level flags */
#define IW_DESCR_FLAG_WAIT	0x0100	/* Wait for driver event */

/****************************** TYPES ******************************/

/* ----------------------- WIRELESS HANDLER ----------------------- */

struct iw_request_info {
	__u16		cmd;		/* Wireless Extension command */
	__u16		flags;		/* More to come ;-) */
};

struct net_device;

typedef int (*iw_handler)(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra);

struct iw_handler_def {

	/* Array of handlers for standard ioctls
	 * We will call dev->wireless_handlers->standard[ioctl - SIOCIWFIRST]
	 */
	const iw_handler *	standard;
	/* Number of handlers defined (more precisely, index of the
	 * last defined handler + 1) */
	__u16			num_standard;

#ifdef CONFIG_WEXT_PRIV
	__u16			num_private;
	/* Number of private arg description */
	__u16			num_private_args;
	/* Array of handlers for private ioctls
	 * Will call dev->wireless_handlers->private[ioctl - SIOCIWFIRSTPRIV]
	 */
	const iw_handler *	private;

	/* Arguments of private handler. This one is just a list, so you
	 * can put it in any order you want and should not leave holes...
	 * We will automatically export that to user space... */
	const struct iw_priv_args *	private_args;
#endif

	/* New location of get_wireless_stats, to de-bloat struct net_device.
	 * The old pointer in struct net_device will be gradually phased
	 * out, and drivers are encouraged to use this one... */
	struct iw_statistics*	(*get_wireless_stats)(struct net_device *dev);
};

/* ---------------------- IOCTL DESCRIPTION ---------------------- */

struct iw_ioctl_description {
	__u8	header_type;		/* NULL, iw_point or other */
	__u8	token_type;		/* Future */
	__u16	token_size;		/* Granularity of payload */
	__u16	min_tokens;		/* Min acceptable token number */
	__u16	max_tokens;		/* Max acceptable token number */
	__u32	flags;			/* Special handling of the request */
};

/* Need to think of short header translation table. Later. */

/* --------------------- ENHANCED SPY SUPPORT --------------------- */

struct iw_spy_data {
	/* --- Standard spy support --- */
	int			spy_number;
	u_char			spy_address[IW_MAX_SPY][ETH_ALEN];
	struct iw_quality	spy_stat[IW_MAX_SPY];
	/* --- Enhanced spy support (event) */
	struct iw_quality	spy_thr_low;	/* Low threshold */
	struct iw_quality	spy_thr_high;	/* High threshold */
	u_char			spy_thr_under[IW_MAX_SPY];
};

/* --------------------- DEVICE WIRELESS DATA --------------------- */
/* Forward declaration */
struct libipw_device;
/* The struct */
struct iw_public_data {
	/* Driver enhanced spy support */
	struct iw_spy_data *		spy_data;
	/* Legacy structure managed by the ipw2x00-specific IEEE 802.11 layer */
	struct libipw_device *		libipw;
};

/**************************** PROTOTYPES ****************************/

/* First : function strictly used inside the kernel */

/* Handle /proc/net/wireless, called in net/code/dev.c */
extern int dev_get_wireless_info(char * buffer, char **start, off_t offset,
				 int length);

/* Second : functions that may be called by driver modules */

/* Send a single event to user space */
extern void wireless_send_event(struct net_device *	dev,
				unsigned int		cmd,
				union iwreq_data *	wrqu,
				const char *		extra);


/* Standard handler for SIOCSIWSPY */
extern int iw_handler_set_spy(struct net_device *	dev,
			      struct iw_request_info *	info,
			      union iwreq_data *	wrqu,
			      char *			extra);
/* Standard handler for SIOCGIWSPY */
extern int iw_handler_get_spy(struct net_device *	dev,
			      struct iw_request_info *	info,
			      union iwreq_data *	wrqu,
			      char *			extra);
/* Standard handler for SIOCSIWTHRSPY */
extern int iw_handler_set_thrspy(struct net_device *	dev,
				 struct iw_request_info *info,
				 union iwreq_data *	wrqu,
				 char *			extra);
/* Standard handler for SIOCGIWTHRSPY */
extern int iw_handler_get_thrspy(struct net_device *	dev,
				 struct iw_request_info *info,
				 union iwreq_data *	wrqu,
				 char *			extra);
/* Driver call to update spy records */
extern void wireless_spy_update(struct net_device *	dev,
				unsigned char *		address,
				struct iw_quality *	wstats);

/************************* INLINE FUNTIONS *************************/

static inline int iwe_stream_lcp_len(struct iw_request_info *info)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT)
		return IW_EV_COMPAT_LCP_LEN;
#endif
	return IW_EV_LCP_LEN;
}

static inline int iwe_stream_point_len(struct iw_request_info *info)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT)
		return IW_EV_COMPAT_POINT_LEN;
#endif
	return IW_EV_POINT_LEN;
}

static inline int iwe_stream_event_len_adjust(struct iw_request_info *info,
					      int event_len)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT) {
		event_len -= IW_EV_LCP_LEN;
		event_len += IW_EV_COMPAT_LCP_LEN;
	}
#endif

	return event_len;
}

/*------------------------------------------------------------------*/
static inline char *
iwe_stream_add_event(struct iw_request_info *info, char *stream, char *ends,
		     struct iw_event *iwe, int event_len)
{
	int lcp_len = iwe_stream_lcp_len(info);

	event_len = iwe_stream_event_len_adjust(info, event_len);

	/* Check if it's possible */
	if(likely((stream + event_len) < ends)) {
		iwe->len = event_len;
		/* Beware of alignement issues on 64 bits */
		memcpy(stream, (char *) iwe, IW_EV_LCP_PK_LEN);
		memcpy(stream + lcp_len, &iwe->u,
		       event_len - lcp_len);
		stream += event_len;
	}
	return stream;
}

/*------------------------------------------------------------------*/
static inline char *
iwe_stream_add_point(struct iw_request_info *info, char *stream, char *ends,
		     struct iw_event *iwe, char *extra)
{
	int event_len = iwe_stream_point_len(info) + iwe->u.data.length;
	int point_len = iwe_stream_point_len(info);
	int lcp_len   = iwe_stream_lcp_len(info);

	/* Check if it's possible */
	if(likely((stream + event_len) < ends)) {
		iwe->len = event_len;
		memcpy(stream, (char *) iwe, IW_EV_LCP_PK_LEN);
		memcpy(stream + lcp_len,
		       ((char *) &iwe->u) + IW_EV_POINT_OFF,
		       IW_EV_POINT_PK_LEN - IW_EV_LCP_PK_LEN);
		memcpy(stream + point_len, extra, iwe->u.data.length);
		stream += event_len;
	}
	return stream;
}

/*------------------------------------------------------------------*/
static inline char *
iwe_stream_add_value(struct iw_request_info *info, char *event, char *value,
		     char *ends, struct iw_event *iwe, int event_len)
{
	int lcp_len = iwe_stream_lcp_len(info);

	/* Don't duplicate LCP */
	event_len -= IW_EV_LCP_LEN;

	/* Check if it's possible */
	if(likely((value + event_len) < ends)) {
		/* Add new value */
		memcpy(value, &iwe->u, event_len);
		value += event_len;
		/* Patch LCP */
		iwe->len = value - event;
		memcpy(event, (char *) iwe, lcp_len);
	}
	return value;
}

#endif	/* _IW_HANDLER_H */
