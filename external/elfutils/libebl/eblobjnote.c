
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <libeblP.h>


void
ebl_object_note (ebl, name, type, descsz, desc)
     Ebl *ebl;
     const char *name;
     uint32_t type;
     uint32_t descsz;
     const char *desc;
{
  if (! ebl->object_note (name, type, descsz, desc))
    /* The machine specific function did not know this type.  */
    switch (type)
      {
      case NT_VERSION:
	if (strcmp (name, "GNU") == 0 && descsz >= 8)
	  {
	    struct
	    {
	      uint32_t os;
	      uint32_t version[descsz / 4 - 1];
	    } *tag = (__typeof (tag)) desc;

	    const char *os;
	    switch (tag->os)
	      {
	      case ELF_NOTE_OS_LINUX:
		os = "Linux";
		break;

	      case ELF_NOTE_OS_GNU:
		os = "GNU";
		break;

	      case ELF_NOTE_OS_SOLARIS2:
		os = "Solaris";
		break;

	      case ELF_NOTE_OS_FREEBSD:
		os = "FreeBSD";
		break;

	      default:
		os = "???";
		break;
	      }

	    printf (gettext ("    OS: %s, ABI: "), os);
	    size_t cnt;
	    for (cnt = 0; cnt < descsz / 4 - 1; ++cnt)
	      {
		if (cnt != 0)
		  putchar_unlocked ('.');
		printf ("%" PRIu32, tag->version[cnt]);
	      }
	    putchar_unlocked ('\n');
	    break;
	  }
	/* FALLTHROUGH */

      default:
	/* Unknown type.  */
	break;
      }
}
