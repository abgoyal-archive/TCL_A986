
/* @(#) $Header: /tcpdump/master/tcpdump/l2tp.h,v 1.5 2001/11/05 10:03:27 guy Exp $ (LBL) */


#define L2TP_FLAG_TYPE		0x8000	/* Type (0=Data, 1=Control) */
#define L2TP_FLAG_LENGTH	0x4000	/* Length */
#define L2TP_FLAG_SEQUENCE	0x0800	/* Sequence */
#define L2TP_FLAG_OFFSET	0x0200	/* Offset */
#define L2TP_FLAG_PRIORITY	0x0100	/* Priority */

#define L2TP_VERSION_MASK	0x000f	/* Version Mask */
#define L2TP_VERSION_L2F	0x0001	/* L2F */
#define L2TP_VERSION_L2TP	0x0002	/* L2TP */

#define L2TP_AVP_HDR_FLAG_MANDATORY	0x8000	/* Mandatory Flag */
#define L2TP_AVP_HDR_FLAG_HIDDEN	0x4000	/* Hidden Flag */
#define L2TP_AVP_HDR_LEN_MASK		0x03ff	/* Length Mask */

#define L2TP_FRAMING_CAP_SYNC_MASK	0x00000001	/* Synchronous */
#define L2TP_FRAMING_CAP_ASYNC_MASK	0x00000002	/* Asynchronous */

#define L2TP_FRAMING_TYPE_SYNC_MASK	0x00000001	/* Synchronous */
#define L2TP_FRAMING_TYPE_ASYNC_MASK	0x00000002	/* Asynchronous */

#define L2TP_BEARER_CAP_DIGITAL_MASK	0x00000001	/* Digital */
#define L2TP_BEARER_CAP_ANALOG_MASK	0x00000002	/* Analog */

#define L2TP_BEARER_TYPE_DIGITAL_MASK	0x00000001	/* Digital */
#define L2TP_BEARER_TYPE_ANALOG_MASK	0x00000002	/* Analog */

/* Authen Type */
#define L2TP_AUTHEN_TYPE_RESERVED	0x0000	/* Reserved */
#define L2TP_AUTHEN_TYPE_TEXTUAL	0x0001	/* Textual username/password exchange */
#define L2TP_AUTHEN_TYPE_CHAP		0x0002	/* PPP CHAP */
#define L2TP_AUTHEN_TYPE_PAP		0x0003	/* PPP PAP */
#define L2TP_AUTHEN_TYPE_NO_AUTH	0x0004	/* No Authentication */
#define L2TP_AUTHEN_TYPE_MSCHAPv1	0x0005	/* MSCHAPv1 */

#define L2TP_PROXY_AUTH_ID_MASK		0x00ff


