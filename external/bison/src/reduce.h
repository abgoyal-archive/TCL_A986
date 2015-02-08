
#ifndef REDUCE_H_
# define REDUCE_H_

void reduce_grammar (void);
void reduce_output (FILE *out);
void reduce_free (void);

extern symbol_number nuseless_nonterminals;
extern rule_number nuseless_productions;
#endif /* !REDUCE_H_ */
