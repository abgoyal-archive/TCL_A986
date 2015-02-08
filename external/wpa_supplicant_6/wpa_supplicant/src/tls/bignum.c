

#include "includes.h"

#include "common.h"
#include "bignum.h"

#ifdef CONFIG_INTERNAL_LIBTOMMATH
#include "libtommath.c"
#else /* CONFIG_INTERNAL_LIBTOMMATH */
#include <tommath.h>
#endif /* CONFIG_INTERNAL_LIBTOMMATH */



struct bignum * bignum_init(void)
{
	struct bignum *n = os_zalloc(sizeof(mp_int));
	if (n == NULL)
		return NULL;
	if (mp_init((mp_int *) n) != MP_OKAY) {
		os_free(n);
		n = NULL;
	}
	return n;
}


void bignum_deinit(struct bignum *n)
{
	if (n) {
		mp_clear((mp_int *) n);
		os_free(n);
	}
}


size_t bignum_get_unsigned_bin_len(struct bignum *n)
{
	return mp_unsigned_bin_size((mp_int *) n);
}


int bignum_get_unsigned_bin(const struct bignum *n, u8 *buf, size_t *len)
{
	size_t need = mp_unsigned_bin_size((mp_int *) n);
	if (len && need > *len) {
		*len = need;
		return -1;
	}
	if (mp_to_unsigned_bin((mp_int *) n, buf) != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	if (len)
		*len = need;
	return 0;
}


int bignum_set_unsigned_bin(struct bignum *n, const u8 *buf, size_t len)
{
	if (mp_read_unsigned_bin((mp_int *) n, (u8 *) buf, len) != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}


int bignum_cmp(const struct bignum *a, const struct bignum *b)
{
	return mp_cmp((mp_int *) a, (mp_int *) b);
}


int bignum_cmp_d(const struct bignum *a, unsigned long b)
{
	return mp_cmp_d((mp_int *) a, b);
}


int bignum_add(const struct bignum *a, const struct bignum *b,
	       struct bignum *c)
{
	if (mp_add((mp_int *) a, (mp_int *) b, (mp_int *) c) != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}


int bignum_sub(const struct bignum *a, const struct bignum *b,
	       struct bignum *c)
{
	if (mp_sub((mp_int *) a, (mp_int *) b, (mp_int *) c) != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}


int bignum_mul(const struct bignum *a, const struct bignum *b,
	       struct bignum *c)
{
	if (mp_mul((mp_int *) a, (mp_int *) b, (mp_int *) c) != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}


int bignum_mulmod(const struct bignum *a, const struct bignum *b,
		  const struct bignum *c, struct bignum *d)
{
	if (mp_mulmod((mp_int *) a, (mp_int *) b, (mp_int *) c, (mp_int *) d)
	    != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}


int bignum_exptmod(const struct bignum *a, const struct bignum *b,
		   const struct bignum *c, struct bignum *d)
{
	if (mp_exptmod((mp_int *) a, (mp_int *) b, (mp_int *) c, (mp_int *) d)
	    != MP_OKAY) {
		wpa_printf(MSG_DEBUG, "BIGNUM: %s failed", __func__);
		return -1;
	}
	return 0;
}
