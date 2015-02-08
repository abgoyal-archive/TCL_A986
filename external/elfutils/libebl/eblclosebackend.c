
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dlfcn.h>
#include <stdlib.h>

#include <libeblP.h>


void
ebl_closebackend (Ebl *ebl)
{
  if (ebl != NULL)
    {
      /* Run the destructor.  */
      ebl->destr (ebl);

      /* Close the dynamically loaded object.  */
      if (ebl->dlhandle != NULL)
	(void) dlclose (ebl->dlhandle);

      /* Free the resources.  */
      free (ebl);
    }
}
