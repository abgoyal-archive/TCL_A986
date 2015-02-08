
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <inttypes.h>
#include <stdio.h>
#include <libeblP.h>


const char *
ebl_core_note_type_name (ebl, type, buf, len)
     Ebl *ebl;
     uint32_t type;
     char *buf;
     size_t len;
{
  const char *res = ebl->core_note_type_name (type, buf, len);

  if (res == NULL)
    {
      static const char *knowntypes[] =
	{
#define KNOWNSTYPE(name) [NT_##name] = #name
	  KNOWNSTYPE (PRSTATUS),
	  KNOWNSTYPE (FPREGSET),
	  KNOWNSTYPE (PRPSINFO),
	  KNOWNSTYPE (TASKSTRUCT),
	  KNOWNSTYPE (PLATFORM),
	  KNOWNSTYPE (AUXV),
	  KNOWNSTYPE (GWINDOWS),
	  KNOWNSTYPE (ASRS),
	  KNOWNSTYPE (PSTATUS),
	  KNOWNSTYPE (PSINFO),
	  KNOWNSTYPE (PRCRED),
	  KNOWNSTYPE (UTSNAME),
	  KNOWNSTYPE (LWPSTATUS),
	  KNOWNSTYPE (LWPSINFO),
	  KNOWNSTYPE (PRFPXREG)
	};

      /* Handle standard names.  */
      if (type < sizeof (knowntypes) / sizeof (knowntypes[0])
	  && knowntypes[type] != NULL)
	res = knowntypes[type];
      else
	{
	  snprintf (buf, len, "%s: %" PRIu32, gettext ("<unknown>"), type);

	  res = buf;
	}
    }

  return res;
}
