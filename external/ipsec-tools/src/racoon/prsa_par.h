/* A Bison parser, made by GNU Bison 2.3.  */



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     COLON = 258,
     HEX = 259,
     OBRACE = 260,
     EBRACE = 261,
     TAG_RSA = 262,
     TAG_PUB = 263,
     TAG_PSK = 264,
     MODULUS = 265,
     PUBLIC_EXPONENT = 266,
     PRIVATE_EXPONENT = 267,
     PRIME1 = 268,
     PRIME2 = 269,
     EXPONENT1 = 270,
     EXPONENT2 = 271,
     COEFFICIENT = 272,
     ADDR4 = 273,
     ADDR6 = 274,
     ADDRANY = 275,
     SLASH = 276,
     NUMBER = 277,
     BASE64 = 278
   };
#endif
/* Tokens.  */
#define COLON 258
#define HEX 259
#define OBRACE 260
#define EBRACE 261
#define TAG_RSA 262
#define TAG_PUB 263
#define TAG_PSK 264
#define MODULUS 265
#define PUBLIC_EXPONENT 266
#define PRIVATE_EXPONENT 267
#define PRIME1 268
#define PRIME2 269
#define EXPONENT1 270
#define EXPONENT2 271
#define COEFFICIENT 272
#define ADDR4 273
#define ADDR6 274
#define ADDRANY 275
#define SLASH 276
#define NUMBER 277
#define BASE64 278




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 130 "prsa_par.y"
{
	BIGNUM *bn;
	RSA *rsa;
	char *chr;
	long num;
	struct netaddr *naddr;
}
/* Line 1489 of yacc.c.  */
#line 103 "prsa_par.h"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE prsalval;

