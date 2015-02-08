/* A Bison parser, made by GNU Bison 2.3.  */



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     DIR = 258,
     PRIORITY = 259,
     PLUS = 260,
     PRIO_BASE = 261,
     PRIO_OFFSET = 262,
     ACTION = 263,
     PROTOCOL = 264,
     MODE = 265,
     LEVEL = 266,
     LEVEL_SPECIFY = 267,
     IPADDRESS = 268,
     PORT = 269,
     ME = 270,
     ANY = 271,
     SLASH = 272,
     HYPHEN = 273
   };
#endif
/* Tokens.  */
#define DIR 258
#define PRIORITY 259
#define PLUS 260
#define PRIO_BASE 261
#define PRIO_OFFSET 262
#define ACTION 263
#define PROTOCOL 264
#define MODE 265
#define LEVEL 266
#define LEVEL_SPECIFY 267
#define IPADDRESS 268
#define PORT 269
#define ME 270
#define ANY 271
#define SLASH 272
#define HYPHEN 273




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 129 "policy_parse.y"
{
	u_int num;
	u_int32_t num32;
	struct _val {
		int len;
		char *buf;
	} val;
}
/* Line 1489 of yacc.c.  */
#line 94 "policy_parse.h"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE __libipseclval;

