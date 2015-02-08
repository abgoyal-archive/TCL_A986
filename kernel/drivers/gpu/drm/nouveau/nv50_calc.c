

#include "drmP.h"
#include "drm_fixed.h"
#include "nouveau_drv.h"
#include "nouveau_hw.h"

int
nv50_calc_pll(struct drm_device *dev, struct pll_lims *pll, int clk,
	      int *N1, int *M1, int *N2, int *M2, int *P)
{
	struct nouveau_pll_vals pll_vals;
	int ret;

	ret = nouveau_calc_pll_mnp(dev, pll, clk, &pll_vals);
	if (ret <= 0)
		return ret;

	*N1 = pll_vals.N1;
	*M1 = pll_vals.M1;
	*N2 = pll_vals.N2;
	*M2 = pll_vals.M2;
	*P = pll_vals.log2P;
	return ret;
}

int
nv50_calc_pll2(struct drm_device *dev, struct pll_lims *pll, int clk,
	       int *N, int *fN, int *M, int *P)
{
	fixed20_12 fb_div, a, b;

	*P = pll->vco1.maxfreq / clk;
	if (*P > pll->max_p)
		*P = pll->max_p;
	if (*P < pll->min_p)
		*P = pll->min_p;

	/* *M = ceil(refclk / pll->vco.max_inputfreq); */
	a.full = dfixed_const(pll->refclk);
	b.full = dfixed_const(pll->vco1.max_inputfreq);
	a.full = dfixed_div(a, b);
	a.full = dfixed_ceil(a);
	*M = dfixed_trunc(a);

	/* fb_div = (vco * *M) / refclk; */
	fb_div.full = dfixed_const(clk * *P);
	fb_div.full = dfixed_mul(fb_div, a);
	a.full = dfixed_const(pll->refclk);
	fb_div.full = dfixed_div(fb_div, a);

	/* *N = floor(fb_div); */
	a.full = dfixed_floor(fb_div);
	*N = dfixed_trunc(fb_div);

	/* *fN = (fmod(fb_div, 1.0) * 8192) - 4096; */
	b.full = dfixed_const(8192);
	a.full = dfixed_mul(a, b);
	fb_div.full = dfixed_mul(fb_div, b);
	fb_div.full = fb_div.full - a.full;
	*fN = dfixed_trunc(fb_div) - 4096;
	*fN &= 0xffff;

	return clk;
}
