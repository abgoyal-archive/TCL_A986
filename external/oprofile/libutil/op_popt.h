
#ifndef OP_POPT_H
#define OP_POPT_H

#include <popt.h>

// not in some versions of popt.h
#ifndef POPT_TABLEEND
#define POPT_TABLEEND { NULL, '\0', 0, 0, 0, NULL, NULL }
#endif

#ifdef __cplusplus
extern "C" {
#endif

poptContext op_poptGetContext(char const * name,
                int argc, char const ** argv,
                struct poptOption const * options, int flags);

#ifdef __cplusplus
}
#endif

#endif /* OP_POPT_H */
