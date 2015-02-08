
#ifndef TABLES_H_
# define TABLES_H_

# include "state.h"


extern int nvectors;

typedef int base_number;
extern base_number *base;
extern base_number base_ninf;

extern unsigned int *conflict_table;
extern unsigned int *conflict_list;
extern int conflict_list_cnt;

extern base_number *table;
extern base_number *check;
extern base_number table_ninf;

extern state_number *yydefgoto;
extern rule_number *yydefact;
extern int high;

void tables_generate (void);
void tables_free (void);

#endif /* !TABLES_H_ */
