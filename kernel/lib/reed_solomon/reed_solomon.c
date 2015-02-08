

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/rslib.h>
#include <linux/slab.h>
#include <linux/mutex.h>

/* This list holds all currently allocated rs control structures */
static LIST_HEAD (rslist);
/* Protection for the list */
static DEFINE_MUTEX(rslistlock);

static struct rs_control *rs_init(int symsize, int gfpoly, int (*gffunc)(int),
                                  int fcr, int prim, int nroots)
{
	struct rs_control *rs;
	int i, j, sr, root, iprim;

	/* Allocate the control structure */
	rs = kmalloc(sizeof (struct rs_control), GFP_KERNEL);
	if (rs == NULL)
		return NULL;

	INIT_LIST_HEAD(&rs->list);

	rs->mm = symsize;
	rs->nn = (1 << symsize) - 1;
	rs->fcr = fcr;
	rs->prim = prim;
	rs->nroots = nroots;
	rs->gfpoly = gfpoly;
	rs->gffunc = gffunc;

	/* Allocate the arrays */
	rs->alpha_to = kmalloc(sizeof(uint16_t) * (rs->nn + 1), GFP_KERNEL);
	if (rs->alpha_to == NULL)
		goto errrs;

	rs->index_of = kmalloc(sizeof(uint16_t) * (rs->nn + 1), GFP_KERNEL);
	if (rs->index_of == NULL)
		goto erralp;

	rs->genpoly = kmalloc(sizeof(uint16_t) * (rs->nroots + 1), GFP_KERNEL);
	if(rs->genpoly == NULL)
		goto erridx;

	/* Generate Galois field lookup tables */
	rs->index_of[0] = rs->nn;	/* log(zero) = -inf */
	rs->alpha_to[rs->nn] = 0;	/* alpha**-inf = 0 */
	if (gfpoly) {
		sr = 1;
		for (i = 0; i < rs->nn; i++) {
			rs->index_of[sr] = i;
			rs->alpha_to[i] = sr;
			sr <<= 1;
			if (sr & (1 << symsize))
				sr ^= gfpoly;
			sr &= rs->nn;
		}
	} else {
		sr = gffunc(0);
		for (i = 0; i < rs->nn; i++) {
			rs->index_of[sr] = i;
			rs->alpha_to[i] = sr;
			sr = gffunc(sr);
		}
	}
	/* If it's not primitive, exit */
	if(sr != rs->alpha_to[0])
		goto errpol;

	/* Find prim-th root of 1, used in decoding */
	for(iprim = 1; (iprim % prim) != 0; iprim += rs->nn);
	/* prim-th root of 1, index form */
	rs->iprim = iprim / prim;

	/* Form RS code generator polynomial from its roots */
	rs->genpoly[0] = 1;
	for (i = 0, root = fcr * prim; i < nroots; i++, root += prim) {
		rs->genpoly[i + 1] = 1;
		/* Multiply rs->genpoly[] by  @**(root + x) */
		for (j = i; j > 0; j--) {
			if (rs->genpoly[j] != 0) {
				rs->genpoly[j] = rs->genpoly[j -1] ^
					rs->alpha_to[rs_modnn(rs,
					rs->index_of[rs->genpoly[j]] + root)];
			} else
				rs->genpoly[j] = rs->genpoly[j - 1];
		}
		/* rs->genpoly[0] can never be zero */
		rs->genpoly[0] =
			rs->alpha_to[rs_modnn(rs,
				rs->index_of[rs->genpoly[0]] + root)];
	}
	/* convert rs->genpoly[] to index form for quicker encoding */
	for (i = 0; i <= nroots; i++)
		rs->genpoly[i] = rs->index_of[rs->genpoly[i]];
	return rs;

	/* Error exit */
errpol:
	kfree(rs->genpoly);
erridx:
	kfree(rs->index_of);
erralp:
	kfree(rs->alpha_to);
errrs:
	kfree(rs);
	return NULL;
}


void free_rs(struct rs_control *rs)
{
	mutex_lock(&rslistlock);
	rs->users--;
	if(!rs->users) {
		list_del(&rs->list);
		kfree(rs->alpha_to);
		kfree(rs->index_of);
		kfree(rs->genpoly);
		kfree(rs);
	}
	mutex_unlock(&rslistlock);
}

static struct rs_control *init_rs_internal(int symsize, int gfpoly,
                                           int (*gffunc)(int), int fcr,
                                           int prim, int nroots)
{
	struct list_head	*tmp;
	struct rs_control	*rs;

	/* Sanity checks */
	if (symsize < 1)
		return NULL;
	if (fcr < 0 || fcr >= (1<<symsize))
    		return NULL;
	if (prim <= 0 || prim >= (1<<symsize))
    		return NULL;
	if (nroots < 0 || nroots >= (1<<symsize))
		return NULL;

	mutex_lock(&rslistlock);

	/* Walk through the list and look for a matching entry */
	list_for_each(tmp, &rslist) {
		rs = list_entry(tmp, struct rs_control, list);
		if (symsize != rs->mm)
			continue;
		if (gfpoly != rs->gfpoly)
			continue;
		if (gffunc != rs->gffunc)
			continue;
		if (fcr != rs->fcr)
			continue;
		if (prim != rs->prim)
			continue;
		if (nroots != rs->nroots)
			continue;
		/* We have a matching one already */
		rs->users++;
		goto out;
	}

	/* Create a new one */
	rs = rs_init(symsize, gfpoly, gffunc, fcr, prim, nroots);
	if (rs) {
		rs->users = 1;
		list_add(&rs->list, &rslist);
	}
out:
	mutex_unlock(&rslistlock);
	return rs;
}

struct rs_control *init_rs(int symsize, int gfpoly, int fcr, int prim,
                           int nroots)
{
	return init_rs_internal(symsize, gfpoly, NULL, fcr, prim, nroots);
}

struct rs_control *init_rs_non_canonical(int symsize, int (*gffunc)(int),
                                         int fcr, int prim, int nroots)
{
	return init_rs_internal(symsize, 0, gffunc, fcr, prim, nroots);
}

#ifdef CONFIG_REED_SOLOMON_ENC8
int encode_rs8(struct rs_control *rs, uint8_t *data, int len, uint16_t *par,
	       uint16_t invmsk)
{
#include "encode_rs.c"
}
EXPORT_SYMBOL_GPL(encode_rs8);
#endif

#ifdef CONFIG_REED_SOLOMON_DEC8
int decode_rs8(struct rs_control *rs, uint8_t *data, uint16_t *par, int len,
	       uint16_t *s, int no_eras, int *eras_pos, uint16_t invmsk,
	       uint16_t *corr)
{
#include "decode_rs.c"
}
EXPORT_SYMBOL_GPL(decode_rs8);
#endif

#ifdef CONFIG_REED_SOLOMON_ENC16
int encode_rs16(struct rs_control *rs, uint16_t *data, int len, uint16_t *par,
	uint16_t invmsk)
{
#include "encode_rs.c"
}
EXPORT_SYMBOL_GPL(encode_rs16);
#endif

#ifdef CONFIG_REED_SOLOMON_DEC16
int decode_rs16(struct rs_control *rs, uint16_t *data, uint16_t *par, int len,
		uint16_t *s, int no_eras, int *eras_pos, uint16_t invmsk,
		uint16_t *corr)
{
#include "decode_rs.c"
}
EXPORT_SYMBOL_GPL(decode_rs16);
#endif

EXPORT_SYMBOL_GPL(init_rs);
EXPORT_SYMBOL_GPL(init_rs_non_canonical);
EXPORT_SYMBOL_GPL(free_rs);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Reed Solomon encoder/decoder");
MODULE_AUTHOR("Phil Karn, Thomas Gleixner");

