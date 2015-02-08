
#include <nlist.h>
#include <stdio.h>
#include <stdlib.h>


int var = 1;

int bss;


int
foo (int a)
{
  return a;
}

int
main (int argc, char *argv[])
{
  struct nlist nl[6] =
  {
    [0] = { .n_name = "var" },
    [1] = { .n_name = "bss" },
    [2] = { .n_name = "main" },
    [3] = { .n_name = "foo" },
    [4] = { .n_name = "not-there" },
    [5] = { .n_name = NULL },
  };
  int cnt;
  int result = 0;

  if (nlist (".libs/test-nlist", nl) != 0
      && nlist ("./test-nlist", nl) != 0)
    {
      puts ("nlist failed");
      exit (1);
    }

  for (cnt = 0; nl[cnt].n_name != NULL; ++cnt)
    {
      if (argc > 1)
	/* For debugging.  */
	printf ("nl[%d].n_name = \"%s\"\n"
		"nl[%d].n_value = %ld\n"
		"nl[%d].n_scnum = %d\n"
		"nl[%d].n_type = %u\n"
		"nl[%d].n_sclass = %d\n"
		"nl[%d].n_numaux = %d\n\n",
		cnt, nl[cnt].n_name,
		cnt, nl[cnt].n_value,
		cnt, nl[cnt].n_scnum,
		cnt, nl[cnt].n_type,
		cnt, nl[cnt].n_sclass,
		cnt, nl[cnt].n_numaux);

      if ((cnt != 4 && nl[cnt].n_value == 0 && nl[cnt].n_scnum == 0
	   && nl[cnt].n_type == 0 && nl[cnt].n_sclass == 0
	   && nl[cnt].n_numaux == 0)
	  || (cnt == 4 && (nl[cnt].n_value != 0 || nl[cnt].n_scnum != 0
			   || nl[cnt].n_type != 0 || nl[cnt].n_sclass != 0
			   || nl[cnt].n_numaux != 0)))
	result = 1;
    }

  return foo (result);
}
