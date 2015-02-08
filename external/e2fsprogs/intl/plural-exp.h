
#ifndef _PLURAL_EXP_H
#define _PLURAL_EXP_H

#ifndef internal_function
# define internal_function
#endif

#ifndef attribute_hidden
# define attribute_hidden
#endif


struct expression
{
  int nargs;			/* Number of arguments.  */
  enum operator
  {
    /* Without arguments:  */
    var,			/* The variable "n".  */
    num,			/* Decimal number.  */
    /* Unary operators:  */
    lnot,			/* Logical NOT.  */
    /* Binary operators:  */
    mult,			/* Multiplication.  */
    divide,			/* Division.  */
    module,			/* Modulo operation.  */
    plus,			/* Addition.  */
    minus,			/* Subtraction.  */
    less_than,			/* Comparison.  */
    greater_than,		/* Comparison.  */
    less_or_equal,		/* Comparison.  */
    greater_or_equal,		/* Comparison.  */
    equal,			/* Comparison for equality.  */
    not_equal,			/* Comparison for inequality.  */
    land,			/* Logical AND.  */
    lor,			/* Logical OR.  */
    /* Ternary operators:  */
    qmop			/* Question mark operator.  */
  } operation;
  union
  {
    unsigned long int num;	/* Number value for `num'.  */
    struct expression *args[3];	/* Up to three arguments.  */
  } val;
};

struct parse_args
{
  const char *cp;
  struct expression *res;
};


#ifdef _LIBC
# define FREE_EXPRESSION __gettext_free_exp
# define PLURAL_PARSE __gettextparse
# define GERMANIC_PLURAL __gettext_germanic_plural
# define EXTRACT_PLURAL_EXPRESSION __gettext_extract_plural
#elif defined (IN_LIBINTL)
# define FREE_EXPRESSION libintl_gettext_free_exp
# define PLURAL_PARSE libintl_gettextparse
# define GERMANIC_PLURAL libintl_gettext_germanic_plural
# define EXTRACT_PLURAL_EXPRESSION libintl_gettext_extract_plural
#else
# define FREE_EXPRESSION free_plural_expression
# define PLURAL_PARSE parse_plural_expression
# define GERMANIC_PLURAL germanic_plural
# define EXTRACT_PLURAL_EXPRESSION extract_plural_expression
#endif

extern void FREE_EXPRESSION (struct expression *exp)
     internal_function;
extern int PLURAL_PARSE (void *arg);
extern struct expression GERMANIC_PLURAL attribute_hidden;
extern void EXTRACT_PLURAL_EXPRESSION (const char *nullentry,
				       struct expression **pluralp,
				       unsigned long int *npluralsp)
     internal_function;

#if !defined (_LIBC) && !defined (IN_LIBINTL)
extern unsigned long int plural_eval (struct expression *pexp,
				      unsigned long int n);
#endif

#endif /* _PLURAL_EXP_H */
