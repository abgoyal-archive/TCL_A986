
/* { dg-do run } */
#include "ffitest.h"

static double return_dbl(double dbl1, float fl2, unsigned int in3, double dbl4)
{
  return dbl1 + fl2 + in3 + dbl4;
}
int main (void)
{
  ffi_cif cif;
  ffi_type *args[MAX_ARGS];
  void *values[MAX_ARGS];
  double dbl1, dbl4, rdbl;
  float fl2;
  unsigned int in3;
  args[0] = &ffi_type_double;
  args[1] = &ffi_type_float;
  args[2] = &ffi_type_uint;
  args[3] = &ffi_type_double;
  values[0] = &dbl1;
  values[1] = &fl2;
  values[2] = &in3;
  values[3] = &dbl4;

  /* Initialize the cif */
  CHECK(ffi_prep_cif(&cif, FFI_DEFAULT_ABI, 4,
		     &ffi_type_double, args) == FFI_OK);
  dbl1 = 127.0;
  fl2 = 128.0;
  in3 = 255;
  dbl4 = 512.7;

  ffi_call(&cif, FFI_FN(return_dbl), &rdbl, values);
  printf ("%f vs %f\n", rdbl, return_dbl(dbl1, fl2, in3, dbl4));
  CHECK(rdbl ==  dbl1 + fl2 + in3 + dbl4);
  exit(0);
}
