
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


char *
elf_strptr (elf, idx, offset)
     Elf *elf;
     size_t idx;
     size_t offset;
{
  if (elf == NULL)
    return NULL;

  if (elf->kind != ELF_K_ELF)
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  rwlock_rdlock (elf->lock);

  char *result = NULL;
  Elf_Scn *strscn;

  /* Find the section in the list.  */
  Elf_ScnList *runp = (elf->class == ELFCLASS32
		       || (offsetof (struct Elf, state.elf32.scns)
			   == offsetof (struct Elf, state.elf64.scns))
		       ? &elf->state.elf32.scns : &elf->state.elf64.scns);
  while (1)
    {
      if (idx < runp->max)
	{
	  if (idx < runp->cnt)
	    strscn = &runp->data[idx];
	  else
	    {
	      __libelf_seterrno (ELF_E_INVALID_INDEX);
	      goto out;
	    }
	  break;
	}

      idx -= runp->max;

      runp = runp->next;
      if (runp == NULL)
	{
	  __libelf_seterrno (ELF_E_INVALID_INDEX);
	  goto out;
	}
    }

  if (elf->class == ELFCLASS32)
    {
      if (unlikely (strscn->shdr.e32->sh_type != SHT_STRTAB))
	{
	  /* This is no string section.  */
	  __libelf_seterrno (ELF_E_INVALID_SECTION);
	  goto out;
	}

      if (unlikely (offset >= strscn->shdr.e32->sh_size))
	{
	  /* The given offset is too big, it is beyond this section.  */
	  __libelf_seterrno (ELF_E_OFFSET_RANGE);
	  goto out;
	}
    }
  else
    {
      if (unlikely (strscn->shdr.e64->sh_type != SHT_STRTAB))
	{
	  /* This is no string section.  */
	  __libelf_seterrno (ELF_E_INVALID_SECTION);
	  goto out;
	}

      if (unlikely (offset >= strscn->shdr.e64->sh_size))
	{
	  /* The given offset is too big, it is beyond this section.  */
	  __libelf_seterrno (ELF_E_OFFSET_RANGE);
	  goto out;
	}
    }

  if (strscn->rawdata_base == NULL
      /* Read the section data.  */
      && __libelf_set_rawdata (strscn) != 0)
    goto out;

  result = &strscn->rawdata_base[offset];

 out:
  rwlock_unlock (elf->lock);

  return result;
}
INTDEF(elf_strptr)
