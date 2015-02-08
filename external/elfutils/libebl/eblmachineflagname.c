
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <string.h>
#include <libeblP.h>


const char *
ebl_machine_flag_name (ebl, flags, buf, len)
     Ebl *ebl;
     Elf64_Word flags;
     char *buf;
     size_t len;
{
  const char *res;

  if (flags == 0)
    res = "";
  else
    {
      char *cp = buf;
      int first = 1;
      const char *machstr;
      size_t machstrlen;

      do
	{
	  if (! first)
	    {
	      if (cp + 1 >= buf + len)
		break;
	      *cp++ = ',';
	    }

	  machstr = ebl != NULL ? ebl->machine_flag_name (&flags) : NULL;
	  if (machstr == NULL)
	    {
	      /* No more known flag.  */
	      snprintf (cp, buf + len - cp, "%#x", flags);
	      break;
	    }

	  machstrlen = strlen (machstr) + 1;
	  if ((size_t) (buf + len - cp) < machstrlen)
	    {
	      *((char *) mempcpy (cp, machstr, buf + len - cp - 1)) = '\0';
	      break;
	    }

	  cp = mempcpy (cp, machstr, machstrlen);

	  first = 0;
	}
      while (flags != 0);

      res = buf;
    }

  return res;
}
