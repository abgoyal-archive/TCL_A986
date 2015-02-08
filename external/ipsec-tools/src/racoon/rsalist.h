/*	$NetBSD: rsalist.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: rsalist.h,v 1.2 2004/07/12 20:43:51 ludvigm Exp */

#ifndef _RSALIST_H
#define _RSALIST_H

#include <netinet/in.h>
#include <openssl/rsa.h>

#include "handler.h"
#include "genlist.h"

enum rsa_key_type {
	RSA_TYPE_ANY = 0,
	RSA_TYPE_PUBLIC,
	RSA_TYPE_PRIVATE
};

struct rsa_key {
	struct netaddr *src;
	struct netaddr *dst;
	RSA *rsa;
};

int rsa_key_insert(struct genlist *list, struct netaddr *src, struct netaddr *dst, RSA *rsa);
void rsa_key_dump(struct genlist *list);

struct genlist *rsa_lookup_keys(struct ph1handle *iph1, int my);
RSA *rsa_try_check_rsasign(vchar_t *source, vchar_t *sig, struct genlist *list);

unsigned long rsa_list_count(struct genlist *list);

int rsa_parse_file(struct genlist *list, const char *fname, enum rsa_key_type type);

#endif /* _RSALIST_H */
