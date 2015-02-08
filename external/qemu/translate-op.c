
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "config.h"

enum {
#define DEF(s, n, copy_size) INDEX_op_ ## s,
#ifdef GEN_TRACE
#include "opc-trace.h"
#else
#include "opc.h"
#endif
#undef DEF
    NB_OPS,
};

#include "dyngen.h"
#ifdef GEN_TRACE
#define   dyngen_code   _trace_dyngen_code
#include "op-trace.h"
#else
#define   dyngen_code  _default_dyngen_code
#include "op.h"
#endif

typedef int (*dyngen_code_func)(uint8_t *gen_code_buf,
                uint16_t *label_offsets, uint16_t *jmp_offsets,
                const uint16_t *opc_buf, const uint32_t *opparam_buf, const long *gen_labels);

extern dyngen_code_func  _dyngen_code;

#ifdef GEN_TRACE

void  qemu_trace_enable_dyngen( void )
{
    _dyngen_code = dyngen_code;
}

#else

void  qemu_trace_disable_dyngen( void )
{
    _dyngen_code = dyngen_code;
}

dyngen_code_func  _dyngen_code = _default_dyngen_code;

#undef  dyngen_code

int dyngen_code(uint8_t *gen_code_buf,
                uint16_t *label_offsets, uint16_t *jmp_offsets,
                const uint16_t *opc_buf, const uint32_t *opparam_buf, const long *gen_labels)
{
    return (*_dyngen_code)(gen_code_buf, label_offsets, jmp_offsets, opc_buf, opparam_buf, gen_labels);
}

#endif


