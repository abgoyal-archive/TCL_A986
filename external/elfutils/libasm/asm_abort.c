
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include <unistd.h>

#include <libasmP.h>
#include <libelf.h>


int
asm_abort (ctx)
     AsmCtx_t *ctx;
{
  if (ctx == NULL)
    /* Something went wrong earlier.  */
    return -1;

  if (likely (! ctx->textp))
    /* First free the ELF file.  We don't care about the result.  */
    (void) elf_end (ctx->out.elf);

  /* Now close the temporary file and remove it.  */
  (void) unlink (ctx->tmp_fname);

  /* Free the resources.  */
  __libasm_finictx (ctx);

  return 0;
}
