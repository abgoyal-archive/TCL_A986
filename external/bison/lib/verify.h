
/* Written by Paul Eggert, Bruno Haible, and Jim Meyering.  */

#ifndef VERIFY_H
# define VERIFY_H 1




# ifdef __cplusplus
template <int w>
  struct verify_type__ { unsigned int verify_error_if_negative_size__: w; };
#  define verify_true(R) \
     (!!sizeof (verify_type__<(R) ? 1 : -1>))
# else
#  define verify_true(R) \
     (!!sizeof \
      (struct { unsigned int verify_error_if_negative_size__: (R) ? 1 : -1; }))
# endif


# define verify(R) extern int (* verify_function__ (void)) [verify_true (R)]

#endif
