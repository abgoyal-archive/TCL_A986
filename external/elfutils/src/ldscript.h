/* A Bison parser, made by GNU Bison 1.875c.  */



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     kADD_OP = 258,
     kALIGN = 259,
     kENTRY = 260,
     kEXCLUDE_FILE = 261,
     kFILENAME = 262,
     kGLOBAL = 263,
     kGROUP = 264,
     kID = 265,
     kINPUT = 266,
     kINTERP = 267,
     kKEEP = 268,
     kLOCAL = 269,
     kMODE = 270,
     kMUL_OP = 271,
     kNUM = 272,
     kOUTPUT_FORMAT = 273,
     kPAGESIZE = 274,
     kPROVIDE = 275,
     kSEARCH_DIR = 276,
     kSEGMENT = 277,
     kSIZEOF_HEADERS = 278,
     kSORT = 279,
     kVERSION = 280,
     kVERSION_SCRIPT = 281,
     ADD_OP = 282,
     MUL_OP = 283
   };
#endif
#define kADD_OP 258
#define kALIGN 259
#define kENTRY 260
#define kEXCLUDE_FILE 261
#define kFILENAME 262
#define kGLOBAL 263
#define kGROUP 264
#define kID 265
#define kINPUT 266
#define kINTERP 267
#define kKEEP 268
#define kLOCAL 269
#define kMODE 270
#define kMUL_OP 271
#define kNUM 272
#define kOUTPUT_FORMAT 273
#define kPAGESIZE 274
#define kPROVIDE 275
#define kSEARCH_DIR 276
#define kSEGMENT 277
#define kSIZEOF_HEADERS 278
#define kSORT 279
#define kVERSION 280
#define kVERSION_SCRIPT 281
#define ADD_OP 282
#define MUL_OP 283




#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 58 "/home/drepper/gnu/elfutils/src/ldscript.y"
typedef union YYSTYPE {
  uintmax_t num;
  enum expression_tag op;
  char *str;
  struct expression *expr;
  struct input_section_name *sectionname;
  struct filemask_section_name *filemask_section_name;
  struct input_rule *input_rule;
  struct output_rule *output_rule;
  struct assignment *assignment;
  struct filename_list *filename_list;
  struct version *version;
  struct id_list *id_list;
} YYSTYPE;
/* Line 1275 of yacc.c.  */
#line 108 "ldscript.h"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE ldlval;



