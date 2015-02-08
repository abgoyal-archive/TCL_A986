
#ifndef UNIQSTR_H_
# define UNIQSTR_H_


typedef char const *uniqstr;

/* Return the uniqstr for STR.  */
uniqstr uniqstr_new (char const *str);

/* Two uniqstr values have the same value iff they are the same.  */
#define UNIQSTR_EQ(USTR1, USTR2) ((USTR1) == (USTR2))


/* Create the string table.  */
void uniqstrs_new (void);

/* Die if STR is not a uniqstr.  */
void uniqstr_assert (char const *str);

/* Free all the memory allocated for symbols.  */
void uniqstrs_free (void);

/* Report them all.  */
void uniqstrs_print (void);

#endif /* ! defined UNIQSTR_H_ */
