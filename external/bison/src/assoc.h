
#ifndef ASSOC_H_
# define ASSOC_H_

/* Associativity values for tokens and rules.  */
typedef enum
{
  undef_assoc,
  right_assoc,
  left_assoc,
  non_assoc
} assoc;

char const *assoc_to_string (assoc a);

#endif /* !ASSOC_H_ */
