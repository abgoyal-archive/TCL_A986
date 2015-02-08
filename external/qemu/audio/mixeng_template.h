


#ifndef SIGNED
#define HALF (IN_MAX >> 1)
#endif

#ifdef CONFIG_MIXEMU
#ifdef FLOAT_MIXENG
#define VOL(a, b) ((a) * (b))
#else
#define VOL(a, b) ((a) * (b)) >> 32
#endif
#else
#define VOL(a, b) a
#endif

#define ET glue (ENDIAN_CONVERSION, glue (_, IN_T))

#ifdef FLOAT_MIXENG
static mixeng_real inline glue (conv_, ET) (IN_T v)
{
    IN_T nv = ENDIAN_CONVERT (v);

#ifdef RECIPROCAL
#ifdef SIGNED
    return nv * (1.f / (mixeng_real) (IN_MAX - IN_MIN));
#else
    return (nv - HALF) * (1.f / (mixeng_real) IN_MAX);
#endif
#else  /* !RECIPROCAL */
#ifdef SIGNED
    return nv / (mixeng_real) (IN_MAX - IN_MIN);
#else
    return (nv - HALF) / (mixeng_real) IN_MAX;
#endif
#endif
}

static IN_T inline glue (clip_, ET) (mixeng_real v)
{
    if (v >= 0.5) {
        return IN_MAX;
    }
    else if (v < -0.5) {
        return IN_MIN;
    }

#ifdef SIGNED
    return ENDIAN_CONVERT ((IN_T) (v * (IN_MAX - IN_MIN)));
#else
    return ENDIAN_CONVERT ((IN_T) ((v * IN_MAX) + HALF));
#endif
}

#else  /* !FLOAT_MIXENG */

static inline int64_t glue (conv_, ET) (IN_T v)
{
    IN_T nv = ENDIAN_CONVERT (v);
#ifdef SIGNED
    return ((int64_t) nv) << (32 - SHIFT);
#else
    return ((int64_t) nv - HALF) << (32 - SHIFT);
#endif
}

static inline IN_T glue (clip_, ET) (int64_t v)
{
    if (v >= 0x7f000000) {
        return IN_MAX;
    }
    else if (v < -2147483648LL) {
        return IN_MIN;
    }

#ifdef SIGNED
    return ENDIAN_CONVERT ((IN_T) (v >> (32 - SHIFT)));
#else
    return ENDIAN_CONVERT ((IN_T) ((v >> (32 - SHIFT)) + HALF));
#endif
}
#endif

static void glue (glue (conv_, ET), _to_stereo)
    (struct st_sample *dst, const void *src, int samples, struct mixeng_volume *vol)
{
    struct st_sample *out = dst;
    IN_T *in = (IN_T *) src;
#ifdef CONFIG_MIXEMU
    if (vol->mute) {
        mixeng_clear (dst, samples);
        return;
    }
#else
    (void) vol;
#endif
    while (samples--) {
        out->l = VOL (glue (conv_, ET) (*in++), vol->l);
        out->r = VOL (glue (conv_, ET) (*in++), vol->r);
        out += 1;
    }
}

static void glue (glue (conv_, ET), _to_mono)
    (struct st_sample *dst, const void *src, int samples, struct mixeng_volume *vol)
{
    struct st_sample *out = dst;
    IN_T *in = (IN_T *) src;
#ifdef CONFIG_MIXEMU
    if (vol->mute) {
        mixeng_clear (dst, samples);
        return;
    }
#else
    (void) vol;
#endif
    while (samples--) {
        out->l = VOL (glue (conv_, ET) (in[0]), vol->l);
        out->r = out->l;
        out += 1;
        in += 1;
    }
}

static void glue (glue (clip_, ET), _from_stereo)
    (void *dst, const struct st_sample *src, int samples)
{
    const struct st_sample *in = src;
    IN_T *out = (IN_T *) dst;
    while (samples--) {
        *out++ = glue (clip_, ET) (in->l);
        *out++ = glue (clip_, ET) (in->r);
        in += 1;
    }
}

static void glue (glue (clip_, ET), _from_mono)
    (void *dst, const struct st_sample *src, int samples)
{
    const struct st_sample *in = src;
    IN_T *out = (IN_T *) dst;
    while (samples--) {
        *out++ = glue (clip_, ET) (in->l + in->r);
        in += 1;
    }
}

#undef ET
#undef HALF
#undef VOL
