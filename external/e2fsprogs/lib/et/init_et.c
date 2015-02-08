
#include <stdio.h>
#include <errno.h>
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#include "com_err.h"
#include "error_table.h"

struct foobar {
    struct et_list etl;
    struct error_table et;
};

extern struct et_list * _et_dynamic_list;

int init_error_table(const char * const *msgs, long base, int count)
{
    struct foobar * new_et;

    if (!base || !count || !msgs)
	return 0;

    new_et = (struct foobar *) malloc(sizeof(struct foobar));
    if (!new_et)
	return ENOMEM;	/* oops */
    new_et->etl.table = &new_et->et;
    new_et->et.msgs = msgs;
    new_et->et.base = base;
    new_et->et.n_msgs= count;

    new_et->etl.next = _et_dynamic_list;
    _et_dynamic_list = &new_et->etl;
    return 0;
}
