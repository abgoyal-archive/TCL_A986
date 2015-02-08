
#ifndef UTIL_H
#define UTIL_H 1

U_CDECL_BEGIN

void get_dirname(char *dirname, const char *filename);
void get_basename(char *basename, const char *filename);
int32_t itostr(char * buffer, int32_t i, uint32_t radix, int32_t pad);
U_CDECL_END
#endif /* ! UTIL_H */

