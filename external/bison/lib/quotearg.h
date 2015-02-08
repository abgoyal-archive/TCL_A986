
/* Written by Paul Eggert <eggert@twinsun.com> */

#ifndef QUOTEARG_H_
# define QUOTEARG_H_ 1

# include <stddef.h>

/* Basic quoting styles.  */
enum quoting_style
  {
    /* Output names as-is (ls --quoting-style=literal).  */
    literal_quoting_style,

    /* Quote names for the shell if they contain shell metacharacters
       or would cause ambiguous output (ls --quoting-style=shell).  */
    shell_quoting_style,

    /* Quote names for the shell, even if they would normally not
       require quoting (ls --quoting-style=shell-always).  */
    shell_always_quoting_style,

    /* Quote names as for a C language string (ls --quoting-style=c).  */
    c_quoting_style,

    /* Like c_quoting_style except omit the surrounding double-quote
       characters (ls --quoting-style=escape).  */
    escape_quoting_style,

    /* Like clocale_quoting_style, but quote `like this' instead of
       "like this" in the default C locale (ls --quoting-style=locale).  */
    locale_quoting_style,

    /* Like c_quoting_style except use quotation marks appropriate for
       the locale (ls --quoting-style=clocale).  */
    clocale_quoting_style
  };

/* For now, --quoting-style=literal is the default, but this may change.  */
# ifndef DEFAULT_QUOTING_STYLE
#  define DEFAULT_QUOTING_STYLE literal_quoting_style
# endif

/* Names of quoting styles and their corresponding values.  */
extern char const *const quoting_style_args[];
extern enum quoting_style const quoting_style_vals[];

struct quoting_options;


struct quoting_options *clone_quoting_options (struct quoting_options *o);

/* Get the value of O's quoting style.  If O is null, use the default.  */
enum quoting_style get_quoting_style (struct quoting_options *o);

void set_quoting_style (struct quoting_options *o, enum quoting_style s);

int set_char_quoting (struct quoting_options *o, char c, int i);

size_t quotearg_buffer (char *buffer, size_t buffersize,
			char const *arg, size_t argsize,
			struct quoting_options const *o);

char *quotearg_alloc (char const *arg, size_t argsize,
		      struct quoting_options const *o);

char *quotearg_n (int n, char const *arg);

/* Equivalent to quotearg_n (0, ARG).  */
char *quotearg (char const *arg);

char *quotearg_n_style (int n, enum quoting_style s, char const *arg);

char *quotearg_n_style_mem (int n, enum quoting_style s,
			    char const *arg, size_t argsize);

/* Equivalent to quotearg_n_style (0, S, ARG).  */
char *quotearg_style (enum quoting_style s, char const *arg);

/* Like quotearg (ARG), except also quote any instances of CH.  */
char *quotearg_char (char const *arg, char ch);

/* Equivalent to quotearg_char (ARG, ':').  */
char *quotearg_colon (char const *arg);

#endif /* !QUOTEARG_H_ */
