

struct	arc_header {
	u_int8_t  arc_shost;
	u_int8_t  arc_dhost;
	u_int8_t  arc_type;
	/*
	 * only present for newstyle encoding with LL fragmentation.
	 * Don't use sizeof(anything), use ARC_HDR{,NEW}LEN instead.
	 */
	u_int8_t  arc_flag;
	u_int16_t arc_seqid;

	/*
	 * only present in exception packets (arc_flag == 0xff)
	 */
	u_int8_t  arc_type2;	/* same as arc_type */
	u_int8_t  arc_flag2;	/* real flag value */
	u_int16_t arc_seqid2;	/* real seqid value */
};

#define	ARC_HDRLEN		3
#define	ARC_HDRNEWLEN		6
#define	ARC_HDRNEWLEN_EXC	10

/* RFC 1051 */
#define	ARCTYPE_IP_OLD		240	/* IP protocol */
#define	ARCTYPE_ARP_OLD		241	/* address resolution protocol */

/* RFC 1201 */
#define	ARCTYPE_IP		212	/* IP protocol */
#define	ARCTYPE_ARP		213	/* address resolution protocol */
#define	ARCTYPE_REVARP		214	/* reverse addr resolution protocol */

#define	ARCTYPE_ATALK		221	/* Appletalk */
#define	ARCTYPE_BANIAN		247	/* Banyan Vines */
#define	ARCTYPE_IPX		250	/* Novell IPX */

#define ARCTYPE_INET6		0xc4	/* IPng */
#define ARCTYPE_DIAGNOSE	0x80	/* as per ANSI/ATA 878.1 */

struct	arc_linux_header {
	u_int8_t  arc_shost;
	u_int8_t  arc_dhost;
	u_int16_t arc_offset;
	u_int8_t  arc_type;
	/*
	 * only present for newstyle encoding with LL fragmentation.
	 * Don't use sizeof(anything), use ARC_LINUX_HDR{,NEW}LEN
	 * instead.
	 */
	u_int8_t  arc_flag;
	u_int16_t arc_seqid;
};

#define	ARC_LINUX_HDRLEN	5
#define	ARC_LINUX_HDRNEWLEN	8
