
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_symbol_binding_name (ebl, binding, buf, len)
     Ebl *ebl;
     int binding;
     char *buf;
     size_t len;
{
  const char *res;

  res = ebl != NULL ? ebl->symbol_type_name (binding, buf, len) : NULL;
  if (res == NULL)
    {
      static const char *stb_names[STB_NUM] =
	{
	  "LOCAL", "GLOBAL", "WEAK"
	};

      /* Standard binding?  */
      if (binding < STB_NUM)
	res = stb_names[binding];
      else
	{
	  if (binding >= STB_LOPROC && binding <= STB_HIPROC)
	    snprintf (buf, len, "LOPROC+%d", binding - STB_LOPROC);
	  else if (binding >= STB_LOOS && binding <= STB_HIOS)
	    snprintf (buf, len, "LOOS+%d", binding - STB_LOOS);
	  else
	    snprintf (buf, len, gettext ("<unknown>: %d"), binding);

	  res = buf;
	}
    }

  return res;
}
