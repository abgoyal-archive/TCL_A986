
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_osabi_name (ebl, osabi, buf, len)
     Ebl *ebl;
     int osabi;
     char *buf;
     size_t len;
{
  const char *res = ebl != NULL ? ebl->osabi_name (osabi, buf, len) : NULL;

  if (res == NULL)
    {
      if (osabi == ELFOSABI_NONE)
	res = "UNIX - System V";
      else if (osabi == ELFOSABI_HPUX)
	res = "HP/UX";
      else if (osabi == ELFOSABI_NETBSD)
	res = "NetBSD";
      else if (osabi == ELFOSABI_LINUX)
	res = "Linux";
      else if (osabi == ELFOSABI_SOLARIS)
	res = "Solaris";
      else if (osabi == ELFOSABI_AIX)
	res = "AIX";
      else if (osabi == ELFOSABI_IRIX)
	res = "Irix";
      else if (osabi == ELFOSABI_FREEBSD)
	res = "FreeBSD";
      else if (osabi == ELFOSABI_TRU64)
	res = "TRU64";
      else if (osabi == ELFOSABI_MODESTO)
	res = "Modesto";
      else if (osabi == ELFOSABI_OPENBSD)
	res = "OpenBSD";
      else if (osabi == ELFOSABI_ARM)
	res = "Arm";
      else if (osabi == ELFOSABI_STANDALONE)
	res = gettext ("Stand alone");
      else
	{
	  snprintf (buf, len, "%s: %d", gettext ("<unknown>"), osabi);

	  res = buf;
	}
    }

  return res;
}
