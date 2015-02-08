
#ifndef QEMU_MIXENG_H
#define QEMU_MIXENG_H

#ifdef FLOAT_MIXENG
typedef float mixeng_real;
struct mixeng_volume { int mute; mixeng_real r; mixeng_real l; };
struct st_sample { mixeng_real l; mixeng_real r; };
#else
struct mixeng_volume { int mute; int64_t r; int64_t l; };
struct st_sample { int64_t l; int64_t r; };
#endif

typedef void (t_sample) (struct st_sample *dst, const void *src,
                         int samples, struct mixeng_volume *vol);
typedef void (f_sample) (void *dst, const struct st_sample *src, int samples);

extern t_sample *mixeng_conv[2][2][2][3];
extern f_sample *mixeng_clip[2][2][2][3];

void *st_rate_start (int inrate, int outrate);
void st_rate_flow (void *opaque, struct st_sample *ibuf, struct st_sample *obuf,
                   int *isamp, int *osamp);
void st_rate_flow_mix (void *opaque, struct st_sample *ibuf, struct st_sample *obuf,
                       int *isamp, int *osamp);
void st_rate_stop (void *opaque);
void mixeng_clear (struct st_sample *buf, int len);

#endif  /* mixeng.h */
