
#ifndef OP_FILEIO_H
#define OP_FILEIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "op_types.h"

#include <stdio.h>

FILE * op_try_open_file(char const * name, char const * mode);

FILE * op_open_file(char const * name, char const * mode);

u32 op_read_int_from_file(char const * filename, int fatal);

void op_close_file(FILE * fp);

void op_write_file(FILE * fp, void const * buf, size_t size);

void op_write_u32(FILE * fp, u32 val);

void op_write_u64(FILE * fp, u64 val);

void op_write_u8(FILE * fp, u8 val);

char * op_get_line(FILE * fp);

unsigned long calc_crc32(unsigned long crc, unsigned char * buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* OP_FILEIO_H */
