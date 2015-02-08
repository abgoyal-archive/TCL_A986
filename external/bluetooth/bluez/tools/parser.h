/* A Bison parser, made by GNU Bison 2.3.  */



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     K_BIND = 258,
     K_DEVICE = 259,
     K_CHANNEL = 260,
     K_COMMENT = 261,
     K_YES = 262,
     K_NO = 263,
     NUMBER = 264,
     RFCOMM = 265,
     STRING = 266,
     WORD = 267,
     BDADDR = 268
   };
#endif
/* Tokens.  */
#define K_BIND 258
#define K_DEVICE 259
#define K_CHANNEL 260
#define K_COMMENT 261
#define K_YES 262
#define K_NO 263
#define NUMBER 264
#define RFCOMM 265
#define STRING 266
#define WORD 267
#define BDADDR 268




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 50 "parser.y"
{
	int number;
	char *string;
	bdaddr_t *bdaddr;
}
/* Line 1489 of yacc.c.  */
#line 81 "parser.h"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE yylval;

