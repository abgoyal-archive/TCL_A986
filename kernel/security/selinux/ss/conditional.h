

#ifndef _CONDITIONAL_H_
#define _CONDITIONAL_H_

#include "avtab.h"
#include "symtab.h"
#include "policydb.h"

#define COND_EXPR_MAXDEPTH 10

struct cond_expr {
#define COND_BOOL	1 /* plain bool */
#define COND_NOT	2 /* !bool */
#define COND_OR		3 /* bool || bool */
#define COND_AND	4 /* bool && bool */
#define COND_XOR	5 /* bool ^ bool */
#define COND_EQ		6 /* bool == bool */
#define COND_NEQ	7 /* bool != bool */
#define COND_LAST	COND_NEQ
	__u32 expr_type;
	__u32 bool;
	struct cond_expr *next;
};

struct cond_av_list {
	struct avtab_node *node;
	struct cond_av_list *next;
};

struct cond_node {
	int cur_state;
	struct cond_expr *expr;
	struct cond_av_list *true_list;
	struct cond_av_list *false_list;
	struct cond_node *next;
};

int cond_policydb_init(struct policydb *p);
void cond_policydb_destroy(struct policydb *p);

int cond_init_bool_indexes(struct policydb *p);
int cond_destroy_bool(void *key, void *datum, void *p);

int cond_index_bool(void *key, void *datum, void *datap);

int cond_read_bool(struct policydb *p, struct hashtab *h, void *fp);
int cond_read_list(struct policydb *p, void *fp);

void cond_compute_av(struct avtab *ctab, struct avtab_key *key, struct av_decision *avd);

int evaluate_cond_node(struct policydb *p, struct cond_node *node);

#endif /* _CONDITIONAL_H_ */
