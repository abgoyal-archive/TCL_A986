
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include <string.h>

#include <libasmP.h>
#include <system.h>


int
asm_fill (asmscn, bytes, len)
     AsmScn_t *asmscn;
     void *bytes;
     size_t len;
{
  struct FillPattern *pattern;
  struct FillPattern *old_pattern;

  if (asmscn == NULL)
    /* Some earlier error.  */
    return -1;

  if (bytes == NULL)
    /* Use the default pattern.  */
    pattern = (struct FillPattern *) __libasm_default_pattern;
  else
    {
      /* Allocate appropriate memory.  */
      pattern = (struct FillPattern *) malloc (sizeof (struct FillPattern)
					       + len);
      if (pattern == NULL)
	return -1;

      pattern->len = len;
      memcpy (pattern->bytes, bytes, len);
    }

  old_pattern = asmscn->pattern;
  asmscn->pattern = pattern;

  /* Free the old data structure if we have allocated it.  */
  if (old_pattern != __libasm_default_pattern)
    free (old_pattern);

  return 0;
}
