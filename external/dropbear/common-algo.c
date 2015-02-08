
#include "algo.h"
#include "dbutil.h"



#ifdef DROPBEAR_AES256_CBC
static const struct dropbear_cipher dropbear_aes256 = 
	{&aes_desc, 32, 16};
#endif
#ifdef DROPBEAR_AES128_CBC
static const struct dropbear_cipher dropbear_aes128 = 
	{&aes_desc, 16, 16};
#endif
#ifdef DROPBEAR_BLOWFISH_CBC
static const struct dropbear_cipher dropbear_blowfish = 
	{&blowfish_desc, 16, 8};
#endif
#ifdef DROPBEAR_TWOFISH256_CBC
static const struct dropbear_cipher dropbear_twofish256 = 
	{&twofish_desc, 32, 16};
#endif
#ifdef DROPBEAR_TWOFISH128_CBC
static const struct dropbear_cipher dropbear_twofish128 = 
	{&twofish_desc, 16, 16};
#endif
#ifdef DROPBEAR_3DES_CBC
static const struct dropbear_cipher dropbear_3des = 
	{&des3_desc, 24, 8};
#endif

/* used to indicate no encryption, as defined in rfc2410 */
const struct dropbear_cipher dropbear_nocipher =
	{NULL, 16, 8}; 


#ifdef DROPBEAR_SHA1_HMAC
static const struct dropbear_hash dropbear_sha1 = 
	{&sha1_desc, 20, 20};
#endif
#ifdef DROPBEAR_SHA1_96_HMAC
static const struct dropbear_hash dropbear_sha1_96 = 
	{&sha1_desc, 20, 12};
#endif
#ifdef DROPBEAR_MD5_HMAC
static const struct dropbear_hash dropbear_md5 = 
	{&md5_desc, 16, 16};
#endif

const struct dropbear_hash dropbear_nohash =
	{NULL, 16, 0}; /* used initially */
	

/* The following map ssh names to internal values */

algo_type sshciphers[] = {
#ifdef DROPBEAR_AES128_CBC
	{"aes128-cbc", 0, (void*)&dropbear_aes128, 1},
#endif
#ifdef DROPBEAR_3DES_CBC
	{"3des-cbc", 0, (void*)&dropbear_3des, 1},
#endif
#ifdef DROPBEAR_AES256_CBC
	{"aes256-cbc", 0, (void*)&dropbear_aes256, 1},
#endif
#ifdef DROPBEAR_TWOFISH256_CBC
	{"twofish256-cbc", 0, (void*)&dropbear_twofish256, 1},
	{"twofish-cbc", 0, (void*)&dropbear_twofish256, 1},
#endif
#ifdef DROPBEAR_TWOFISH128_CBC
	{"twofish128-cbc", 0, (void*)&dropbear_twofish128, 1},
#endif
#ifdef DROPBEAR_BLOWFISH_CBC
	{"blowfish-cbc", 0, (void*)&dropbear_blowfish, 1},
#endif
	{NULL, 0, NULL, 0}
};

algo_type sshhashes[] = {
#ifdef DROPBEAR_SHA1_96_HMAC
	{"hmac-sha1-96", 0, (void*)&dropbear_sha1_96, 1},
#endif
#ifdef DROPBEAR_SHA1_HMAC
	{"hmac-sha1", 0, (void*)&dropbear_sha1, 1},
#endif
#ifdef DROPBEAR_MD5_HMAC
	{"hmac-md5", 0, (void*)&dropbear_md5, 1},
#endif
	{NULL, 0, NULL, 0}
};

algo_type sshcompress[] = {
#ifndef DISABLE_ZLIB
	{"zlib", DROPBEAR_COMP_ZLIB, NULL, 1},
#endif
	{"none", DROPBEAR_COMP_NONE, NULL, 1},
	{NULL, 0, NULL, 0}
};

algo_type sshhostkey[] = {
#ifdef DROPBEAR_RSA
	{"ssh-rsa", DROPBEAR_SIGNKEY_RSA, NULL, 1},
#endif
#ifdef DROPBEAR_DSS
	{"ssh-dss", DROPBEAR_SIGNKEY_DSS, NULL, 1},
#endif
	{NULL, 0, NULL, 0}
};

algo_type sshkex[] = {
	{"diffie-hellman-group1-sha1", DROPBEAR_KEX_DH_GROUP1, NULL, 1},
	{NULL, 0, NULL, 0}
};


void crypto_init() {

	const struct ltc_cipher_descriptor *regciphers[] = {
#ifdef DROPBEAR_AES_CBC
		&aes_desc,
#endif
#ifdef DROPBEAR_BLOWFISH_CBC
		&blowfish_desc,
#endif
#ifdef DROPBEAR_TWOFISH_CBC
		&twofish_desc,
#endif
#ifdef DROPBEAR_3DES_CBC
		&des3_desc,
#endif
		NULL
	};

	const struct ltc_hash_descriptor *reghashes[] = {
		/* we need sha1 for hostkey stuff regardless */
		&sha1_desc,
#ifdef DROPBEAR_MD5_HMAC
		&md5_desc,
#endif
		NULL
	};	
	int i;
	
	for (i = 0; regciphers[i] != NULL; i++) {
		if (register_cipher(regciphers[i]) == -1) {
			dropbear_exit("error registering crypto");
		}
	}

	for (i = 0; reghashes[i] != NULL; i++) {
		if (register_hash(reghashes[i]) == -1) {
			dropbear_exit("error registering crypto");
		}
	}
}

int have_algo(char* algo, size_t algolen, algo_type algos[]) {

	int i;

	for (i = 0; algos[i].name != NULL; i++) {
		if (strlen(algos[i].name) == algolen
				&& (strncmp(algos[i].name, algo, algolen) == 0)) {
			return DROPBEAR_SUCCESS;
		}
	}

	return DROPBEAR_FAILURE;
}



/* Output a comma separated list of algorithms to a buffer */
void buf_put_algolist(buffer * buf, algo_type localalgos[]) {

	unsigned int i, len;
	unsigned int donefirst = 0;
	buffer *algolist = NULL;

	algolist = buf_new(100);
	for (i = 0; localalgos[i].name != NULL; i++) {
		if (localalgos[i].usable) {
			if (donefirst)
				buf_putbyte(algolist, ',');
			donefirst = 1;
			len = strlen(localalgos[i].name);
			buf_putbytes(algolist, localalgos[i].name, len);
		}
	}
	buf_putstring(buf, algolist->data, algolist->len);
	buf_free(algolist);
}
