

#ifndef RELATION_H_
# define RELATION_H_


#define END_NODE ((relation_node) -1)

typedef size_t relation_node;
typedef relation_node *relation_nodes;
typedef relation_nodes *relation;


/* Report a relation R that has SIZE vertices.  */
void relation_print (relation r, relation_node size, FILE *out);

void relation_digraph (relation r, relation_node size, bitsetv *function);

/* Destructively transpose *R_ARG, of size N.  */
void relation_transpose (relation *R_arg, relation_node n);

#endif /* ! RELATION_H_ */
