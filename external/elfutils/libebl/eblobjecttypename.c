
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_object_type_name (ebl, object, buf, len)
     Ebl *ebl;
     int object;
     char *buf;
     size_t len;
{
  const char *res;

  res = ebl != NULL ? ebl->object_type_name (object, buf, len) : NULL;
  if (res == NULL)
    {
      /* Handle OS-specific section names.  */
      if (object >= ET_LOOS && object <= ET_HIOS)
	snprintf (buf, len, "LOOS+%x", object - ET_LOOS);
      /* Handle processor-specific section names.  */
      else if (object >= ET_LOPROC && object <= ET_HIPROC)
	snprintf (buf, len, "LOPROC+%x", object - ET_LOPROC);
      else
	snprintf (buf, len, "%s: %d", gettext ("<unknown>"), object);

      res = buf;
    }

  return res;
}
