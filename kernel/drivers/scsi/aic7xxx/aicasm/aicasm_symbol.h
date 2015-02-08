

#ifdef __linux__
#include "../queue.h"
#else
#include <sys/queue.h>
#endif

typedef enum {
	UNINITIALIZED,
	REGISTER,
	ALIAS,
	SCBLOC,
	SRAMLOC,
	ENUM_ENTRY,
	FIELD,
	MASK,
	ENUM,
	CONST,
	DOWNLOAD_CONST,
	LABEL,
	CONDITIONAL,
	MACRO
} symtype;

typedef enum {
	RO = 0x01,
	WO = 0x02,
	RW = 0x03
}amode_t;

typedef SLIST_HEAD(symlist, symbol_node) symlist_t;

struct reg_info {
	u_int	  address;
	int	  size;
	amode_t	  mode;
	symlist_t fields;
	uint8_t	  valid_bitmask;
	uint8_t	  modes;
	int	  typecheck_masks;
};

struct field_info {
	symlist_t symrefs;
	uint8_t	  value;
	uint8_t	  mask;
};

struct const_info {
	u_int	value;
	int	define;
};

struct alias_info {
	struct symbol *parent;
};

struct label_info {
	int	address;
	int	exported;
};

struct cond_info {
	int	func_num;
};

struct macro_arg {
	STAILQ_ENTRY(macro_arg)	links;
	regex_t	arg_regex;
	char   *replacement_text;
};
STAILQ_HEAD(macro_arg_list, macro_arg) args;

struct macro_info {
	struct macro_arg_list args;
	int   narg;
	const char* body;
};

typedef struct expression_info {
        symlist_t       referenced_syms;
        int             value;
} expression_t;

typedef struct symbol {
	char	*name;
	symtype	type;
	int	count;
	union	{
		struct reg_info	  *rinfo;
		struct field_info *finfo;
		struct const_info *cinfo;
		struct alias_info *ainfo;
		struct label_info *linfo;
		struct cond_info  *condinfo;
		struct macro_info *macroinfo;
	} info;
	int	dont_generate_debug_code;
} symbol_t;

typedef struct symbol_ref {
	symbol_t *symbol;
	int	 offset;
} symbol_ref_t;

typedef struct symbol_node {
	SLIST_ENTRY(symbol_node) links;
	symbol_t *symbol;
} symbol_node_t;

typedef struct critical_section {
	TAILQ_ENTRY(critical_section) links;
	int begin_addr;
	int end_addr;
} critical_section_t;

typedef enum {
	SCOPE_ROOT,
	SCOPE_IF,
	SCOPE_ELSE_IF,
	SCOPE_ELSE
} scope_type;

typedef struct patch_info {
	int skip_patch;
	int skip_instr;
} patch_info_t;

typedef struct scope {
	SLIST_ENTRY(scope) scope_stack_links;
	TAILQ_ENTRY(scope) scope_links;
	TAILQ_HEAD(, scope) inner_scope;
	scope_type type;
	int inner_scope_patches;
	int begin_addr;
        int end_addr;
	patch_info_t patches[2];
	int func_num;
} scope_t;

TAILQ_HEAD(cs_tailq, critical_section);
SLIST_HEAD(scope_list, scope);
TAILQ_HEAD(scope_tailq, scope);

void	symbol_delete(symbol_t *symbol);

void	symtable_open(void);

void	symtable_close(void);

symbol_t *
	symtable_get(char *name);

symbol_node_t *
	symlist_search(symlist_t *symlist, char *symname);

void
	symlist_add(symlist_t *symlist, symbol_t *symbol, int how);
#define SYMLIST_INSERT_HEAD	0x00
#define SYMLIST_SORT		0x01

void	symlist_free(symlist_t *symlist);

void	symlist_merge(symlist_t *symlist_dest, symlist_t *symlist_src1,
		      symlist_t *symlist_src2);
void	symtable_dump(FILE *ofile, FILE *dfile);
