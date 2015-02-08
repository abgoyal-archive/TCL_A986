
#include <fcntl.h>
#include <libelf.h>
#include <libdw.h>
#include <stdio.h>
#include <unistd.h>


static int globcnt;

static int
callback (Dwarf *dbg, Dwarf_Global *gl, void *arg)
{
  int result = DWARF_CB_OK;

  printf (" [%2d] \"%s\", die: %llu, cu: %llu\n",
	  globcnt++, gl->name, (unsigned long long int) gl->die_offset,
	  (unsigned long long int) gl->cu_offset);

  Dwarf_Die cu_die;
  const char *cuname;
  if (dwarf_offdie (dbg, gl->cu_offset, &cu_die) == NULL
      || (cuname = dwarf_diename (&cu_die)) == NULL)
    {
      puts ("failed to get CU die");
      result = DWARF_CB_ABORT;
    }
  else
    printf ("CU name: \"%s\"\n", cuname);

  const char *diename;
  Dwarf_Die die;
  if (dwarf_offdie (dbg, gl->die_offset, &die) == NULL
      || (diename = dwarf_diename (&die)) == NULL)
    {
      puts ("failed to get object die");
      result = DWARF_CB_ABORT;
    }
  else
    printf ("object name: \"%s\"\n", diename);

  return result;
}


int
main (int argc, char *argv[])
{
  int result = 0;
  int cnt;

  for (cnt = 1; cnt < argc; ++cnt)
    {
      int fd = open (argv[cnt], O_RDONLY);
      Dwarf *dbg = dwarf_begin (fd, DWARF_C_READ);
      if (dbg == NULL)
	{
	  printf ("%s not usable: %s\n", argv[cnt], dwarf_errmsg (-1));
	  result = 1;
	  close (fd);
	  continue;
	}

      globcnt = 0;

      if (dwarf_getpubnames (dbg, callback, NULL, 0) != 0)
	{
	  printf ("dwarf_get_pubnames didn't return zero: %s\n",
		  dwarf_errmsg (-1));
	  result = 1;
	}

      dwarf_end (dbg);
      close (fd);
    }

  return result;
}
