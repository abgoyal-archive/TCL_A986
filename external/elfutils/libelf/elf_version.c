
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelfP.h>


/* Is the version initialized?  */
int __libelf_version_initialized;

/* Currently selected version.  */
unsigned int __libelf_version = EV_CURRENT;


unsigned int
elf_version (version)
     unsigned int version;
{
  if (version == EV_NONE)
    return __libelf_version;

  if (likely (version < EV_NUM))
    {
      /* Phew, we know this version.  */
      unsigned int last_version = __libelf_version;

      /* Store the new version.  */
      __libelf_version = version;

      /* Signal that the version is now initialized.  */
      __libelf_version_initialized = 1;

      /* And return the last version.  */
      return last_version;
    }

  /* We cannot handle this version.  */
  __libelf_seterrno (ELF_E_UNKNOWN_VERSION);
  return EV_NONE;
}
INTDEF(elf_version)
