
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "com_err.h"
#include "error_table.h"

const char *
com_right(struct et_list *list, long code)
{
    struct et_list *p;
    for (p = list; p; p = p->next) {
	if (code >= p->table->base && code < p->table->base + p->table->n_msgs)
	    return p->table->msgs[code - p->table->base];
    }
    return NULL;
}

struct foobar {
    struct et_list etl;
    struct error_table tab;
};

void
initialize_error_table_r(struct et_list **list, 
			 const char **messages, 
			 int num_errors,
			 long base)
{
    struct et_list *et, **end;
    struct error_table *tab;
    struct foobar *f;

    for (end = list, et = *list; et; end = &et->next, et = et->next)
        if (et->table->msgs == messages)
            return;
    f = malloc(sizeof(*f));
    if (f == NULL)
        return;
    et = &f->etl;
    et->table = tab = &f->tab;
    tab->msgs = messages;
    tab->n_msgs = num_errors;
    tab->base = base;
    et->next = NULL;
    *end = et;
}
			

void
free_error_table(struct et_list *et)
{
    while(et){
	struct et_list *p = et;
	et = et->next;
	free(p);
    }
}
