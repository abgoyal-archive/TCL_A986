

#ifdef __linux__
#include "../queue.h"
#else
#include <sys/queue.h>
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct path_entry {
	char	*directory;
	int	quoted_includes_only;
	SLIST_ENTRY(path_entry) links;
} *path_entry_t;

typedef enum {  
	QUOTED_INCLUDE,
	BRACKETED_INCLUDE,
	SOURCE_FILE
} include_type;

SLIST_HEAD(path_list, path_entry);

extern struct path_list search_path;
extern struct cs_tailq cs_tailq;
extern struct scope_list scope_stack;
extern struct symlist patch_functions;
extern int includes_search_curdir;		/* False if we've seen -I- */
extern char *appname;
extern char *stock_include_file;
extern int yylineno;
extern char *yyfilename;
extern char *prefix;
extern char *patch_arg_list;
extern char *versions;
extern int   src_mode;
extern int   dst_mode;
struct symbol;

void stop(const char *errstring, int err_code);
void include_file(char *file_name, include_type type);
void expand_macro(struct symbol *macro_symbol);
struct instruction *seq_alloc(void);
struct critical_section *cs_alloc(void);
struct scope *scope_alloc(void);
void process_scope(struct scope *);
