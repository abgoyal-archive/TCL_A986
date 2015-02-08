


#include <stdio.h>

struct dtc_file {
	char *dir;
	const char *name;
	FILE *file;
};

#if ! defined(YYLTYPE) && ! defined(YYLTYPE_IS_DECLARED)
typedef struct YYLTYPE {
    int first_line;
    int first_column;
    int last_line;
    int last_column;
    struct dtc_file *file;
} YYLTYPE;

#define YYLTYPE_IS_DECLARED	1
#define YYLTYPE_IS_TRIVIAL	1
#endif

/* Cater to old parser templates. */
#ifndef YYID
#define YYID(n)	(n)
#endif

#define YYLLOC_DEFAULT(Current, Rhs, N)					\
    do									\
      if (YYID (N))							\
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	  (Current).file         = YYRHSLOC (Rhs, N).file;		\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	  (Current).file         = YYRHSLOC (Rhs, 0).file;		\
	}								\
    while (YYID (0))



extern void yyerror(char const *);
extern void yyerrorf(char const *, ...) __attribute__((format(printf, 1, 2)));

extern struct dtc_file *srcpos_file;

struct search_path {
	const char *dir; /* NULL for current directory */
	struct search_path *prev, *next;
};

extern struct dtc_file *dtc_open_file(const char *fname,
                                      const struct search_path *search);
extern void dtc_close_file(struct dtc_file *file);
