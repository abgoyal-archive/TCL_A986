
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stddef.h>

#include <libasmP.h>


Elf *
asm_getelf (ctx)
     AsmCtx_t *ctx;
{
  return ctx != NULL ? ctx->out.elf : NULL;
}
