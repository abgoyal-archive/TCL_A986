/*	$NetBSD: netdb_dnssec.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: netdb_dnssec.h,v 1.3 2004/06/11 16:00:17 ludvigm Exp */


#ifndef _NETDB_DNSSEC_H
#define _NETDB_DNSSEC_H

#ifndef T_CERT
#define T_CERT	37		/* defined by RFC2538 section 2 */
#endif

/* RFC2538 section 2.1 */
#define DNSSEC_TYPE_PKIX	1
#define DNSSEC_TYPE_SPKI	2
#define DNSSEC_TYPE_PGP		3
#define DNSSEC_TYPE_URI		4
#define DNSSEC_TYPE_OID		5

/* RFC2535 section 3.2 */
#define DNSSEC_ALG_RSAMD5	1
#define DNSSEC_ALG_DH		2
#define DNSSEC_ALG_DSA		3
#define DNSSEC_ALG_ECC		4
#define DNSSEC_ALG_PRIVATEDNS	5
#define DNSSEC_ALG_PRIVATEOID	6

struct certinfo {
	int ci_type;			/* certificate type */
	int ci_keytag;			/* keytag */
	int ci_algorithm;		/* algorithm */
	int ci_flags;			/* currently, 1:valid or 0:uncertain */
	size_t ci_certlen;		/* length of certificate */
	char *ci_cert;			/* certificate */
	struct certinfo *ci_next;	/* next structure */
};

extern void freecertinfo __P((struct certinfo *));
extern int getcertsbyname __P((char *, struct certinfo **));

#endif /* _NETDB_DNSSEC_H */
