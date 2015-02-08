/*	$OpenBSD: nameser.h,v 1.11 2005/12/20 02:06:56 millert Exp $	*/



#ifndef _NAMESER_H_
#define _NAMESER_H_

#include <sys/cdefs.h>
#include <sys/param.h>


#define __BIND		19960801	/* interface version stamp */

#define PACKETSZ	512		/* maximum packet size */
#define MAXDNAME	1025		/* maximum presentation domain name */
#define MAXCDNAME	255		/* maximum compressed domain name */
#define MAXLABEL	63		/* maximum length of domain label */
#define HFIXEDSZ	12		/* #/bytes of fixed data in header */
#define QFIXEDSZ	4		/* #/bytes of fixed data in query */
#define RRFIXEDSZ	10		/* #/bytes of fixed data in r record */
#define INT32SZ		4		/* for systems without 32-bit ints */
#define INT16SZ		2		/* for systems without 16-bit ints */
#define INADDRSZ	4		/* IPv4 T_A */
#define IN6ADDRSZ	16		/* IPv6 T_AAAA */

#define NAMESERVER_PORT	53

#define QUERY		0x0		/* standard query */
#define IQUERY		0x1		/* inverse query */
#define STATUS		0x2		/* nameserver status query */
/*#define xxx		0x3*/		/* 0x3 reserved */
#define NS_NOTIFY_OP	0x4		/* notify secondary of SOA change */
#define NOERROR		0		/* no error */
#define FORMERR		1		/* format error */
#define SERVFAIL	2		/* server failure */
#define NXDOMAIN	3		/* non existent domain */
#define NOTIMP		4		/* not implemented */
#define REFUSED		5		/* query refused */

#define T_A		1		/* host address */
#define T_NS		2		/* authoritative server */
#define T_MD		3		/* mail destination */
#define T_MF		4		/* mail forwarder */
#define T_CNAME		5		/* canonical name */
#define T_SOA		6		/* start of authority zone */
#define T_MB		7		/* mailbox domain name */
#define T_MG		8		/* mail group member */
#define T_MR		9		/* mail rename name */
#define T_NULL		10		/* null resource record */
#define T_WKS		11		/* well known service */
#define T_PTR		12		/* domain name pointer */
#define T_HINFO		13		/* host information */
#define T_MINFO		14		/* mailbox information */
#define T_MX		15		/* mail routing information */
#define T_TXT		16		/* text strings */
#define T_RP		17		/* responsible person */
#define T_AFSDB		18		/* AFS cell database */
#define T_X25		19		/* X_25 calling address */
#define T_ISDN		20		/* ISDN calling address */
#define T_RT		21		/* router */
#define T_NSAP		22		/* NSAP address */
#define T_NSAP_PTR	23		/* reverse NSAP lookup (deprecated) */
#define T_SIG		24		/* security signature */
#define T_KEY		25		/* security key */
#define T_PX		26		/* X.400 mail mapping */
#define T_GPOS		27		/* geographical position (withdrawn) */
#define T_AAAA		28		/* IP6 Address */
#define T_LOC		29		/* Location Information */
#define T_NXT		30		/* Next Valid Name in Zone */
#define T_EID		31		/* Endpoint identifier */
#define T_NIMLOC	32		/* Nimrod locator */
#define T_SRV		33		/* Server selection */
#define T_ATMA		34		/* ATM Address */
#define T_NAPTR		35		/* Naming Authority PoinTeR */
#define T_KX		36		/* Key Exchanger */
#define T_CERT		37		/* CERT */
#define T_A6		38		/* A6 */
#define T_DNAME		39		/* DNAME */
#define T_SINK		40		/* SINK */
#define T_OPT		41		/* OPT pseudo-RR, RFC2671 */
#define T_APL		42		/* APL */
#define T_DS		43		/* Delegation Signer */
#define T_SSHFP		44		/* SSH Key Fingerprint */
#define T_RRSIG		46		/* RRSIG */
#define T_NSEC		47		/* NSEC */
#define T_DNSKEY	48		/* DNSKEY */
	/* non standard */
#define T_UINFO		100		/* user (finger) information */
#define T_UID		101		/* user ID */
#define T_GID		102		/* group ID */
#define T_UNSPEC	103		/* Unspecified format (binary data) */
	/* Query type values which do not appear in resource records */
#define	T_TKEY		249		/* Transaction Key */
#define	T_TSIG		250		/* Transaction Signature */
#define	T_IXFR		251		/* incremental zone transfer */
#define T_AXFR		252		/* transfer zone of authority */
#define T_MAILB		253		/* transfer mailbox records */
#define T_MAILA		254		/* transfer mail agent records */
#define T_ANY		255		/* wildcard match */


#define C_IN		1		/* the arpa internet */
#define C_CHAOS		3		/* for chaos net (MIT) */
#define C_HS		4		/* for Hesiod name server (MIT) (XXX) */
	/* Query class values which do not appear in resource records */
#define C_ANY		255		/* wildcard match */

#define	KEYFLAG_TYPEMASK	0xC000	/* Mask for "type" bits */
#define	KEYFLAG_TYPE_AUTH_CONF	0x0000	/* Key usable for both */
#define	KEYFLAG_TYPE_CONF_ONLY	0x8000	/* Key usable for confidentiality */
#define	KEYFLAG_TYPE_AUTH_ONLY	0x4000	/* Key usable for authentication */
#define	KEYFLAG_TYPE_NO_KEY	0xC000	/* No key usable for either; no key */
/* The type bits can also be interpreted independently, as single bits: */
#define	KEYFLAG_NO_AUTH		0x8000	/* Key not usable for authentication */
#define	KEYFLAG_NO_CONF		0x4000	/* Key not usable for confidentiality */

#define	KEYFLAG_EXPERIMENTAL	0x2000	/* Security is *mandatory* if bit=0 */
#define	KEYFLAG_RESERVED3	0x1000  /* reserved - must be zero */
#define	KEYFLAG_RESERVED4	0x0800  /* reserved - must be zero */
#define	KEYFLAG_USERACCOUNT	0x0400	/* key is assoc. with a user acct */
#define	KEYFLAG_ENTITY		0x0200	/* key is assoc. with entity eg host */
#define	KEYFLAG_ZONEKEY		0x0100	/* key is zone key for the zone named */
#define	KEYFLAG_IPSEC		0x0080  /* key is for IPSEC use (host or user)*/
#define	KEYFLAG_EMAIL		0x0040  /* key is for email (MIME security) */
#define	KEYFLAG_RESERVED10	0x0020  /* reserved - must be zero */
#define	KEYFLAG_RESERVED11	0x0010  /* reserved - must be zero */
#define	KEYFLAG_SIGNATORYMASK	0x000F	/* key can sign DNS RR's of same name */

#define  KEYFLAG_RESERVED_BITMASK ( KEYFLAG_RESERVED3 | \
				    KEYFLAG_RESERVED4 | \
				    KEYFLAG_RESERVED10| KEYFLAG_RESERVED11) 

/* The Algorithm field of the KEY and SIG RR's is an integer, {1..254} */
#define	ALGORITHM_MD5RSA	1	/* MD5 with RSA */
#define	ALGORITHM_EXPIRE_ONLY	253	/* No alg, no security */
#define	ALGORITHM_PRIVATE_OID	254	/* Key begins with OID indicating alg */

/* Signatures */
					/* Size of a mod or exp in bits */
#define	MIN_MD5RSA_KEY_PART_BITS	 512
#define	MAX_MD5RSA_KEY_PART_BITS	2552
					/* Total of binary mod and exp, bytes */
#define	MAX_MD5RSA_KEY_BYTES		((MAX_MD5RSA_KEY_PART_BITS+7/8)*2+3)
					/* Max length of text sig block */
#define	MAX_KEY_BASE64			(((MAX_MD5RSA_KEY_BYTES+2)/3)*4)

#define DNS_MESSAGEEXTFLAG_DO	0x8000U

#define CONV_SUCCESS	0
#define CONV_OVERFLOW	(-1)
#define CONV_BADFMT	(-2)
#define CONV_BADCKSUM	(-3)
#define CONV_BADBUFLEN	(-4)

#if !defined(_BYTE_ORDER) || \
    (_BYTE_ORDER != _BIG_ENDIAN && _BYTE_ORDER != _LITTLE_ENDIAN && \
    _BYTE_ORDER != _PDP_ENDIAN)
	/* you must determine what the correct bit order is for
	 * your compiler - the next line is an intentional error
	 * which will force your compiles to bomb until you fix
	 * the above macros.
	 */
#error "Undefined or invalid _BYTE_ORDER";
#endif


typedef struct {
	unsigned	id :16;		/* query identification number */
#if _BYTE_ORDER == _BIG_ENDIAN
			/* fields in third byte */
	unsigned	qr: 1;		/* response flag */
	unsigned	opcode: 4;	/* purpose of message */
	unsigned	aa: 1;		/* authoritive answer */
	unsigned	tc: 1;		/* truncated message */
	unsigned	rd: 1;		/* recursion desired */
			/* fields in fourth byte */
	unsigned	ra: 1;		/* recursion available */
	unsigned	unused :1;	/* unused bits (MBZ as of 4.9.3a3) */
	unsigned	ad: 1;		/* authentic data from named */
	unsigned	cd: 1;		/* checking disabled by resolver */
	unsigned	rcode :4;	/* response code */
#endif
#if _BYTE_ORDER == _LITTLE_ENDIAN || _BYTE_ORDER == _PDP_ENDIAN
			/* fields in third byte */
	unsigned	rd :1;		/* recursion desired */
	unsigned	tc :1;		/* truncated message */
	unsigned	aa :1;		/* authoritive answer */
	unsigned	opcode :4;	/* purpose of message */
	unsigned	qr :1;		/* response flag */
			/* fields in fourth byte */
	unsigned	rcode :4;	/* response code */
	unsigned	cd: 1;		/* checking disabled by resolver */
	unsigned	ad: 1;		/* authentic data from named */
	unsigned	unused :1;	/* unused bits (MBZ as of 4.9.3a3) */
	unsigned	ra :1;		/* recursion available */
#endif
			/* remaining bytes */
	unsigned	qdcount :16;	/* number of question entries */
	unsigned	ancount :16;	/* number of answer entries */
	unsigned	nscount :16;	/* number of authority entries */
	unsigned	arcount :16;	/* number of resource entries */
} HEADER;

#define INDIR_MASK	0xc0

extern	u_int16_t	_getshort(const unsigned char *);
extern	u_int32_t	_getlong(const unsigned char *);

#define GETSHORT(s, cp) { \
	unsigned char *t_cp = (unsigned char *)(cp); \
	(s) = ((u_int16_t)t_cp[0] << 8) \
	    | ((u_int16_t)t_cp[1]) \
	    ; \
	(cp) += INT16SZ; \
}

#define GETLONG(l, cp) { \
	unsigned char *t_cp = (unsigned char *)(cp); \
	(l) = ((u_int32_t)t_cp[0] << 24) \
	    | ((u_int32_t)t_cp[1] << 16) \
	    | ((u_int32_t)t_cp[2] << 8) \
	    | ((u_int32_t)t_cp[3]) \
	    ; \
	(cp) += INT32SZ; \
}

#define PUTSHORT(s, cp) { \
	u_int16_t t_s = (u_int16_t)(s); \
	unsigned char *t_cp = (unsigned char *)(cp); \
	*t_cp++ = t_s >> 8; \
	*t_cp   = t_s; \
	(cp) += INT16SZ; \
}

#define PUTLONG(l, cp) { \
	u_int32_t t_l = (u_int32_t)(l); \
	unsigned char *t_cp = (unsigned char *)(cp); \
	*t_cp++ = t_l >> 24; \
	*t_cp++ = t_l >> 16; \
	*t_cp++ = t_l >> 8; \
	*t_cp   = t_l; \
	(cp) += INT32SZ; \
}

#endif /* !_NAMESER_H_ */
