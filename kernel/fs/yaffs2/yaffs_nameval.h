
#ifndef __NAMEVAL_H__
#define __NAMEVAL_H__

#include "yportenv.h"

int nval_del(char *xb, int xb_size, const YCHAR *name);
int nval_set(char *xb, int xb_size, const YCHAR *name, const char *buf, int bsize, int flags);
int nval_get(const char *xb, int xb_size, const YCHAR *name, char *buf, int bsize);
int nval_list(const char *xb, int xb_size, char *buf, int bsize);
int nval_hasvalues(const char *xb, int xb_size);
#endif
