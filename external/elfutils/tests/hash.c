
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stdio.h>


static int
check (const char *name, unsigned long int expected)
{
  unsigned long int actual = elf_hash (name);

  return actual != expected;
}


int
main (void)
{
  int status;

  /* Check some names.  We know what the expected result is.  */
  status = check ("_DYNAMIC", 165832675);
  status |= check ("_GLOBAL_OFFSET_TABLE_", 102264335);

  return status;
}
