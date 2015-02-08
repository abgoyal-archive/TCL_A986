
#ifndef OPD_MANGLING_H
#define OPD_MANGLING_H

#include "odb.h"

struct sfile;

int opd_open_sample_file(odb_t *file, struct sfile *last,
                         struct sfile * sf, int counter, int cg);

#endif /* OPD_MANGLING_H */
