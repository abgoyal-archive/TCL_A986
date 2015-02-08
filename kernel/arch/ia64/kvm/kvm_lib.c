
#undef CONFIG_MODULES
#include <linux/module.h>
#undef CONFIG_KALLSYMS
#undef EXPORT_SYMBOL
#undef EXPORT_SYMBOL_GPL
#define EXPORT_SYMBOL(sym)
#define EXPORT_SYMBOL_GPL(sym)
#include "../../../lib/vsprintf.c"
#include "../../../lib/ctype.c"
