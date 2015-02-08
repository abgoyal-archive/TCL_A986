
#ifndef SYMTAB_H_
# define SYMTAB_H_

# include "assoc.h"
# include "location.h"
# include "uniqstr.h"


/* Symbol classes.  */
typedef enum
{
  unknown_sym,
  token_sym,		/* terminal symbol */
  nterm_sym		/* non-terminal */
} symbol_class;


/* Internal token numbers. */
typedef int symbol_number;
#define SYMBOL_NUMBER_MAXIMUM INT_MAX


typedef struct symbol symbol;

struct symbol
{
  /* The key, name of the symbol.  */
  uniqstr tag;
  /* The location of its first occurrence.  */
  location location;

  /* Its %type and associated printer and destructor.  */
  uniqstr type_name;
  location type_location;

  /* Does not own the memory. */
  const char *destructor;
  location destructor_location;

  /* Does not own the memory. */
  const char *printer;
  location printer_location;

  symbol_number number;
  location prec_location;
  int prec;
  assoc assoc;
  int user_token_number;

  /* Points to the other in the identifier-symbol pair for an alias.
     Special value USER_NUMBER_ALIAS in the identifier half of the
     identifier-symbol pair for an alias.  */
  symbol *alias;
  symbol_class class;
  bool declared;
};

/* Undefined user number.  */
#define USER_NUMBER_UNDEFINED -1

#define USER_NUMBER_ALIAS -9991

/* Undefined internal token number.  */
#define NUMBER_UNDEFINED (-1)

/* Print a symbol (for debugging). */
void symbol_print (symbol *s, FILE *f);

/* Fetch (or create) the symbol associated to KEY.  */
symbol *symbol_get (const char *key, location loc);

symbol *dummy_symbol_get (location loc);

/* Declare the new symbol SYM.  Make it an alias of SYMVAL.  */
void symbol_make_alias (symbol *sym, symbol *symval, location loc);

void symbol_type_set (symbol *sym, uniqstr type_name, location loc);

/* Set the DESTRUCTOR associated with SYM.  */
void symbol_destructor_set (symbol *sym, const char *destructor, location loc);

/* Set the PRINTER associated with SYM.  */
void symbol_printer_set (symbol *sym, const char *printer, location loc);

void symbol_precedence_set (symbol *sym, int prec, assoc a, location loc);

/* Set the CLASS associated with SYM.  */
void symbol_class_set (symbol *sym, symbol_class class, location loc,
		       bool declaring);

/* Set the USER_TOKEN_NUMBER associated with SYM.  */
void symbol_user_token_number_set (symbol *sym, int user_number, location loc);


extern symbol *errtoken;
extern symbol *undeftoken;
extern symbol *endtoken;
extern symbol *accept;
extern symbol *startsymbol;
extern location startsymbol_location;




/* Create the symbol table.  */
void symbols_new (void);

/* Free all the memory allocated for symbols.  */
void symbols_free (void);

void symbols_check_defined (void);

void symbols_pack (void);

#endif /* !SYMTAB_H_ */
