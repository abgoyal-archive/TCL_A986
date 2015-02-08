
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_segment_type_name (ebl, segment, buf, len)
     Ebl *ebl;
     int segment;
     char *buf;
     size_t len;
{
  const char *res;

  res = ebl != NULL ? ebl->segment_type_name (segment, buf, len) : NULL;
  if (res == NULL)
    {
      static const char *ptypes[PT_NUM] =
	{
#define PTYPE(name) [PT_##name] = #name
	  PTYPE (NULL),
	  PTYPE (LOAD),
	  PTYPE (DYNAMIC),
	  PTYPE (INTERP),
	  PTYPE (NOTE),
	  PTYPE (SHLIB),
	  PTYPE (PHDR),
	  PTYPE (TLS)
	};

      /* Is it one of the standard segment types?  */
      if (segment >= PT_NULL && segment < PT_NUM)
	res = ptypes[segment];
      else if (segment == PT_GNU_EH_FRAME)
	res = "GNU_EH_FRAME";
      else if (segment == PT_GNU_STACK)
	res = "GNU_STACK";
      else if (segment == PT_GNU_RELRO)
	res = "GNU_RELRO";
      else if (segment == PT_SUNWBSS)
	res = "SUNWBSS";
      else if (segment == PT_SUNWSTACK)
	res = "SUNWSTACK";
      else
	{
	  if (segment >= PT_LOOS && segment <= PT_HIOS)
	    snprintf (buf, len, "LOOS+%d", segment - PT_LOOS);
	  else if (segment >= PT_LOPROC && segment <= PT_HIPROC)
	    snprintf (buf, len, "LOPROC+%d", segment - PT_LOPROC);
	  else
	    snprintf (buf, len, "%s: %d", gettext ("<unknown>"), segment);

	  res = buf;
	}
    }

  return res;
}
