
#ifndef OP_STRING_H
#define OP_STRING_H

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

char * op_xstrndup(char const * s, size_t len);

size_t op_hash_string(char const * s);

int strisprefix(char const * str, char const * prefix);

char const * skip_ws(char const * c);

char const * skip_nonws(char const * c);

int empty_line(char const * c);

int comment_line(char const * c);

#ifdef __cplusplus
}
#endif

#endif /* !OP_STRING_H */
