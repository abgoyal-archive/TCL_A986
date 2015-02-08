
#ifndef FILES_H_
# define FILES_H_

# include "uniqstr.h"

/* File name specified with -o for the output file, or 0 if no -o.  */
extern char const *spec_outfile;

/* File name for the parser (i.e., the one above, or its default.) */
extern char const *parser_file_name;

/* Symbol prefix specified with -p, or 0 if no -p.  */
extern const char *spec_name_prefix;

/* File name prefix specified with -b, or 0 if no -b.  */
extern char const *spec_file_prefix;

/* --verbose. */
extern char const *spec_verbose_file;

/* File name specified for the output VCG graph.  */
extern char const *spec_graph_file;

/* File name specified with --defines.  */
extern char const *spec_defines_file;

/* Directory prefix of output file names.  */
extern char const *dir_prefix;


/* If semantic parser, output a .h file that defines YYSTYPE... */
extern struct obstack pre_prologue_obstack;
extern struct obstack post_prologue_obstack;

extern uniqstr grammar_file;

/* The current file name.  Might change with %include, or with #line.  */
extern uniqstr current_file;

void compute_output_file_names (void);

FILE *xfopen (const char *name, const char *mode);
void xfclose (FILE *ptr);

#endif /* !FILES_H_ */
