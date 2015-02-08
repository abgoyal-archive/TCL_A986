
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libasmP.h>

#ifndef SIZE
# define SIZE 8
#endif

#define UFCT(size) _UFCT(size)
#define _UFCT(size) asm_adduint##size
#define FCT(size) _FCT(size)
#define _FCT(size) asm_addint##size
#define UTYPE(size) _UTYPE(size)
#define _UTYPE(size) uint##size##_t
#define TYPE(size) _TYPE(size)
#define _TYPE(size) int##size##_t


int
UFCT(SIZE) (asmscn, num)
     AsmScn_t *asmscn;
     UTYPE(SIZE) num;
{
  return INTUSE(FCT(SIZE)) (asmscn, (TYPE(SIZE)) num);
}
