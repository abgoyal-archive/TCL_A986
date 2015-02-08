
#include "includes.h"
#include "dbutil.h"
#include "bignum.h"
#include "random.h"
#include "rsa.h"
#include "genrsa.h"

#define RSA_E 65537

#ifdef DROPBEAR_RSA

static void getrsaprime(mp_int* prime, mp_int *primeminus, 
		mp_int* rsa_e, unsigned int size);

/* mostly taken from libtomcrypt's rsa key generation routine */
rsa_key * gen_rsa_priv_key(unsigned int size) {

	rsa_key * key;
	DEF_MP_INT(pminus);
	DEF_MP_INT(qminus);
	DEF_MP_INT(lcm);

	key = (rsa_key*)m_malloc(sizeof(rsa_key));

	key->e = (mp_int*)m_malloc(sizeof(mp_int));
	key->n = (mp_int*)m_malloc(sizeof(mp_int));
	key->d = (mp_int*)m_malloc(sizeof(mp_int));
	key->p = (mp_int*)m_malloc(sizeof(mp_int));
	key->q = (mp_int*)m_malloc(sizeof(mp_int));

	m_mp_init_multi(key->e, key->n, key->d, key->p, key->q,
			&pminus, &lcm, &qminus, NULL);

	seedrandom();

	if (mp_set_int(key->e, RSA_E) != MP_OKAY) {
		fprintf(stderr, "rsa generation failed\n");
		exit(1);
	}

	/* PuTTY doesn't like it if the modulus isn't a multiple of 8 bits,
	 * so we just generate them until we get one which is OK */
	getrsaprime(key->p, &pminus, key->e, size/2);
	do {
		getrsaprime(key->q, &qminus, key->e, size/2);

		if (mp_mul(key->p, key->q, key->n) != MP_OKAY) {
			fprintf(stderr, "rsa generation failed\n");
			exit(1);
		}
	} while (mp_count_bits(key->n) % 8 != 0);

	/* lcm(p-1, q-1) */
	if (mp_lcm(&pminus, &qminus, &lcm) != MP_OKAY) {
		fprintf(stderr, "rsa generation failed\n");
		exit(1);
	}

	/* de = 1 mod lcm(p-1,q-1) */
	/* therefore d = (e^-1) mod lcm(p-1,q-1) */
	if (mp_invmod(key->e, &lcm, key->d) != MP_OKAY) {
		fprintf(stderr, "rsa generation failed\n");
		exit(1);
	}

	mp_clear_multi(&pminus, &qminus, &lcm, NULL);

	return key;
}	

/* return a prime suitable for p or q */
static void getrsaprime(mp_int* prime, mp_int *primeminus, 
		mp_int* rsa_e, unsigned int size) {

	unsigned char *buf;
	DEF_MP_INT(temp_gcd);

	buf = (unsigned char*)m_malloc(size+1);

	m_mp_init(&temp_gcd);
	do {
		/* generate a random odd number with MSB set, then find the
		   the next prime above it */
		genrandom(buf, size+1);
		buf[0] |= 0x80; /* MSB set */

		bytes_to_mp(prime, buf, size+1);

		/* find the next integer which is prime, 8 round of miller-rabin */
		if (mp_prime_next_prime(prime, 8, 0) != MP_OKAY) {
			fprintf(stderr, "rsa generation failed\n");
			exit(1);
		}

		/* subtract one to get p-1 */
		if (mp_sub_d(prime, 1, primeminus) != MP_OKAY) {
			fprintf(stderr, "rsa generation failed\n");
			exit(1);
		}
		/* check relative primality to e */
		if (mp_gcd(primeminus, rsa_e, &temp_gcd) != MP_OKAY) {
			fprintf(stderr, "rsa generation failed\n");
			exit(1);
		}
	} while (mp_cmp_d(&temp_gcd, 1) != MP_EQ); /* while gcd(p-1, e) != 1 */

	/* now we have a good value for result */
	mp_clear(&temp_gcd);
	m_burn(buf, size+1);
	m_free(buf);
}

#endif /* DROPBEAR_RSA */
