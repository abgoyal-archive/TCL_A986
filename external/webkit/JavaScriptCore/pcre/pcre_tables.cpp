


#include "config.h"
#include "pcre_internal.h"



const int jsc_pcre_utf8_table1[6] =
  { 0x7f, 0x7ff, 0xffff, 0x1fffff, 0x3ffffff, 0x7fffffff};


const int jsc_pcre_utf8_table2[6] = { 0,    0xc0, 0xe0, 0xf0, 0xf8, 0xfc};
const int jsc_pcre_utf8_table3[6] = { 0xff, 0x1f, 0x0f, 0x07, 0x03, 0x01};


const unsigned char jsc_pcre_utf8_table4[0x40] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,4,4,4,4,5,5,5,5 };

#include "chartables.c"
