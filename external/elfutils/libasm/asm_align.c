
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include <sys/param.h>

#include <libasmP.h>
#include <system.h>


int
asm_align (asmscn, value)
     AsmScn_t *asmscn;
     GElf_Word value;
{
  if (asmscn == NULL)
    /* An earlier error.  */
    return -1;

      /* The alignment value must be a power of two.  */
  if (unlikely (! powerof2 (value)))
    {
      __libasm_seterrno (ASM_E_INVALID);
      return -1;
    }

  rwlock_wrlock (asmscn->ctx->lock);

  int result = 0;

  /* Fillbytes necessary?  */
  if ((asmscn->offset & (value - 1)) != 0)
    {
      /* Add fillbytes.  */
      size_t cnt;
      size_t byteptr;

      cnt = value - (asmscn->offset & (value - 1));

      /* Ensure there is enough room to add the fill bytes.  */
      result = __libasm_ensure_section_space (asmscn, cnt);
      if (result != 0)
	goto out;

      /* Fill in the bytes.  We align the pattern according to the
	 current offset.  */
      byteptr = asmscn->offset % asmscn->pattern->len;

      /* Update the total size.  */
      asmscn->offset += cnt;

      do
	{
	  asmscn->content->data[asmscn->content->len++]
	    = asmscn->pattern->bytes[byteptr++];

	  if (byteptr == asmscn->pattern->len)
	    byteptr = 0;
	}
      while (--cnt > 0);
    }

  /* Remember the maximum alignment for this subsection.  */
  if (asmscn->max_align < value)
    {
      asmscn->max_align = value;

      /* Update the parent as well (if it exists).  */
      if (asmscn->subsection_id != 0)
	{
	  rwlock_wrlock (asmscn->data.up->ctx->lock);

	  if (asmscn->data.up->max_align < value)
	    asmscn->data.up->max_align = value;

	  rwlock_unlock (asmscn->data.up->ctx->lock);
	}
    }

 out:
  rwlock_unlock (asmscn->ctx->lock);

  return result;
}


int
__libasm_ensure_section_space (asmscn, len)
     AsmScn_t *asmscn;
     size_t len;
{
  /* The blocks with the section content are kept in a circular
     single-linked list.  */
  size_t size;

  if (asmscn->content == NULL)
    {
      /* This is the first block.  */
      size = MAX (2 * len, 960);

      asmscn->content = (struct AsmData *) malloc (sizeof (struct AsmData)
						   + size);
      if (asmscn->content == NULL)
	return -1;

      asmscn->content->next = asmscn->content;
    }
  else
    {
      struct AsmData *newp;

      if (asmscn->content->maxlen - asmscn->content->len >= len)
	/* Nothing to do, there is enough space.  */
	return 0;

      size = MAX (2 *len, MIN (32768, 2 * asmscn->offset));

      newp = (struct AsmData *) malloc (sizeof (struct AsmData) + size);
      if (newp == NULL)
	return -1;

      newp->next = asmscn->content->next;
      asmscn->content = asmscn->content->next = newp;
    }

  asmscn->content->len = 0;
  asmscn->content->maxlen = size;

  return 0;
}
