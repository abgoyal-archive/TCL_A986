
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <error.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/param.h>
#include "libdwP.h"


void *
__libdw_allocate (Dwarf *dbg, size_t minsize)
{
  size_t size = MAX (dbg->mem_default_size,
		     2 * minsize + offsetof (struct libdw_memblock, mem));
  struct libdw_memblock *newp = malloc (size);
  if (newp == NULL)
    dbg->oom_handler ();

  newp->size = newp->remaining = size - offsetof (struct libdw_memblock, mem);

  newp->prev = dbg->mem_tail;
  dbg->mem_tail = newp;

  return newp->mem;
}


Dwarf_OOM
dwarf_new_oom_handler (Dwarf *dbg, Dwarf_OOM handler)
{
  Dwarf_OOM old = dbg->oom_handler;
  dbg->oom_handler = handler;
  return old;
}


void
__attribute ((noreturn, visibility ("hidden")))
__libdw_oom (void)
{
  while (1)
    error (EXIT_FAILURE, ENOMEM, "libdw");
}
