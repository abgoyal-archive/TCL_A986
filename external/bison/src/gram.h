
#ifndef GRAM_H_
# define GRAM_H_


# include "location.h"
# include "symtab.h"

# define ISTOKEN(i)	((i) < ntokens)
# define ISVAR(i)	((i) >= ntokens)

extern int nsyms;
extern int ntokens;
extern int nvars;

typedef int item_number;
extern item_number *ritem;
extern unsigned int nritems;


static inline item_number
symbol_number_as_item_number (symbol_number sym)
{
  return sym;
}

static inline symbol_number
item_number_as_symbol_number (item_number i)
{
  return i;
}

static inline bool
item_number_is_symbol_number (item_number i)
{
  return i >= 0;
}

/* Rule numbers.  */
typedef int rule_number;
extern rule_number nrules;

static inline item_number
rule_number_as_item_number (rule_number r)
{
  return -1 - r;
}

static inline rule_number
item_number_as_rule_number (item_number i)
{
  return -1 - i;
}

static inline bool
item_number_is_rule_number (item_number i)
{
  return i < 0;
}


typedef struct
{
  /* The number of the rule in the source.  It is usually the index in
     RULES too, except if there are useless rules.  */
  rule_number user_number;

  /* The index in RULES.  Usually the rule number in the source,
     except if some rules are useless.  */
  rule_number number;

  symbol *lhs;
  item_number *rhs;

  /* This symbol provides both the associativity, and the precedence. */
  symbol *prec;

  int dprec;
  int merger;

  /* This symbol was attached to the rule via %prec. */
  symbol *precsym;

  location location;
  bool useful;

  const char *action;
  location action_location;
} rule;

extern rule *rules;

/* A function that selects a rule.  */
typedef bool (*rule_filter) (rule *);

/* Return true IFF the rule has a `number' smaller than NRULES.  */
bool rule_useful_p (rule *r);

/* Return true IFF the rule has a `number' higher than NRULES.  */
bool rule_useless_p (rule *r);

bool rule_never_reduced_p (rule *r);

void rule_lhs_print (rule *r, symbol *previous_lhs, FILE *out);

/* Return the length of the RHS.  */
int rule_rhs_length (rule *r);

/* Print this rule's RHS on OUT.  */
void rule_rhs_print (rule *r, FILE *out);

/* Print this rule on OUT.  */
void rule_print (rule *r, FILE *out);




/* Table of the symbols, indexed by the symbol number. */
extern symbol **symbols;

extern symbol_number *token_translations;
extern int max_user_token_number;



/* Dump RITEM for traces. */
void ritem_print (FILE *out);

/* Return the size of the longest rule RHS.  */
size_t ritem_longest_rhs (void);

void grammar_rules_partial_print (FILE *out, const char *title,
				  rule_filter filter);

/* Print the grammar's rules on OUT.  */
void grammar_rules_print (FILE *out);

/* Dump the grammar. */
void grammar_dump (FILE *out, const char *title);

void grammar_rules_never_reduced_report (const char *message);

/* Free the packed grammar. */
void grammar_free (void);

#endif /* !GRAM_H_ */
