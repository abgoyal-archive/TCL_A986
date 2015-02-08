

#ifndef PWC_DEC23_H
#define PWC_DEC23_H

#include "pwc.h"

struct pwc_dec23_private
{
  unsigned int scalebits;
  unsigned int nbitsmask, nbits; /* Number of bits of a color in the compressed stream */

  unsigned int reservoir;
  unsigned int nbits_in_reservoir;
  const unsigned char *stream;
  int temp_colors[16];

  unsigned char table_0004_pass1[16][1024];
  unsigned char table_0004_pass2[16][1024];
  unsigned char table_8004_pass1[16][256];
  unsigned char table_8004_pass2[16][256];
  unsigned int  table_subblock[256][12];

  unsigned char table_bitpowermask[8][256];
  unsigned int  table_d800[256];
  unsigned int  table_dc00[256];

};


int pwc_dec23_alloc(struct pwc_device *pwc);
int pwc_dec23_init(struct pwc_device *pwc, int type, unsigned char *cmd);
void pwc_dec23_exit(void);
void pwc_dec23_decompress(const struct pwc_device *pwc,
			  const void *src,
			  void *dst,
			  int flags);



#endif


/* vim: set cino= formatoptions=croql cindent shiftwidth=8 tabstop=8: */

