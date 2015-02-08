

void NAME (void *opaque, struct st_sample *ibuf, struct st_sample *obuf,
           int *isamp, int *osamp)
{
    struct rate *rate = opaque;
    struct st_sample *istart, *iend;
    struct st_sample *ostart, *oend;
    struct st_sample ilast, icur, out;
#ifdef FLOAT_MIXENG
    mixeng_real t;
#else
    int64_t t;
#endif

    ilast = rate->ilast;

    istart = ibuf;
    iend = ibuf + *isamp;

    ostart = obuf;
    oend = obuf + *osamp;

    if (rate->opos_inc == (1ULL + UINT_MAX)) {
        int i, n = *isamp > *osamp ? *osamp : *isamp;
        for (i = 0; i < n; i++) {
            OP (obuf[i].l, ibuf[i].l);
            OP (obuf[i].r, ibuf[i].r);
        }
        *isamp = n;
        *osamp = n;
        return;
    }

    while (obuf < oend) {

        /* Safety catch to make sure we have input samples.  */
        if (ibuf >= iend) {
            break;
        }

        /* read as many input samples so that ipos > opos */

        while (rate->ipos <= (rate->opos >> 32)) {
            ilast = *ibuf++;
            rate->ipos++;
            /* See if we finished the input buffer yet */
            if (ibuf >= iend) {
                goto the_end;
            }
        }

        icur = *ibuf;

        /* interpolate */
#ifdef FLOAT_MIXENG
#ifdef RECIPROCAL
        t = (rate->opos & UINT_MAX) * (1.f / UINT_MAX);
#else
        t = (rate->opos & UINT_MAX) / (mixeng_real) UINT_MAX;
#endif
        out.l = (ilast.l * (1.0 - t)) + icur.l * t;
        out.r = (ilast.r * (1.0 - t)) + icur.r * t;
#else
        t = rate->opos & 0xffffffff;
        out.l = (ilast.l * ((int64_t) UINT_MAX - t) + icur.l * t) >> 32;
        out.r = (ilast.r * ((int64_t) UINT_MAX - t) + icur.r * t) >> 32;
#endif

        /* output sample & increment position */
        OP (obuf->l, out.l);
        OP (obuf->r, out.r);
        obuf += 1;
        rate->opos += rate->opos_inc;
    }

the_end:
    *isamp = ibuf - istart;
    *osamp = obuf - ostart;
    rate->ilast = ilast;
}

#undef NAME
#undef OP
