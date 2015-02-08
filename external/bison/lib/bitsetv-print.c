
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include "bitsetv-print.h"


void
bitsetv_matrix_dump (FILE * out, const char *title, bitsetv bset)
{
  bitset_bindex i, j;
  bitset_bindex hsize = bitset_size (bset[0]);

  /* Title. */
  fprintf (out, "%s BEGIN\n", title);

  /* Column numbers. */
  fputs ("   ", out);
  for (i = 0; i < hsize; ++i)
    putc (i / 10 ? '0' + i / 10 : ' ', out);
  putc ('\n', out);
  fputs ("   ", out);
  for (i = 0; i < hsize; ++i)
    fprintf (out, "%d", (int) (i % 10));
  putc ('\n', out);

  /* Bar. */
  fputs ("  .", out);
  for (i = 0; i < hsize; ++i)
    putc ('-', out);
  fputs (".\n", out);

  /* Contents. */
  for (i = 0; bset[i]; ++i)
    {
      fprintf (out, "%2lu|", (unsigned long int) i);
      for (j = 0; j < hsize; ++j)
	fputs (bitset_test (bset[i], j) ? "1" : " ", out);
      fputs ("|\n", out);
    }

  /* Bar. */
  fputs ("  `", out);
  for (i = 0; i < hsize; ++i)
    putc ('-', out);
  fputs ("'\n", out);

  /* End title. */
  fprintf (out, "%s END\n\n", title);
}
