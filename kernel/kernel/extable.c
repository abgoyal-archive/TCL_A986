
#include <linux/ftrace.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/init.h>

#include <asm/sections.h>
#include <asm/uaccess.h>

DEFINE_MUTEX(text_mutex);

extern struct exception_table_entry __start___ex_table[];
extern struct exception_table_entry __stop___ex_table[];

/* Sort the kernel's built-in exception table */
void __init sort_main_extable(void)
{
	sort_extable(__start___ex_table, __stop___ex_table);
}

/* Given an address, look for it in the exception tables. */
const struct exception_table_entry *search_exception_tables(unsigned long addr)
{
	const struct exception_table_entry *e;

	e = search_extable(__start___ex_table, __stop___ex_table-1, addr);
	if (!e)
		e = search_module_extables(addr);
	return e;
}

static inline int init_kernel_text(unsigned long addr)
{
	if (addr >= (unsigned long)_sinittext &&
	    addr <= (unsigned long)_einittext)
		return 1;
	return 0;
}

int core_kernel_text(unsigned long addr)
{
	if (addr >= (unsigned long)_stext &&
	    addr <= (unsigned long)_etext)
		return 1;

	if (system_state == SYSTEM_BOOTING &&
	    init_kernel_text(addr))
		return 1;
	return 0;
}

int __kernel_text_address(unsigned long addr)
{
	if (core_kernel_text(addr))
		return 1;
	if (is_module_text_address(addr))
		return 1;
	/*
	 * There might be init symbols in saved stacktraces.
	 * Give those symbols a chance to be printed in
	 * backtraces (such as lockdep traces).
	 *
	 * Since we are after the module-symbols check, there's
	 * no danger of address overlap:
	 */
	if (init_kernel_text(addr))
		return 1;
	return 0;
}

int kernel_text_address(unsigned long addr)
{
	if (core_kernel_text(addr))
		return 1;
	return is_module_text_address(addr);
}

int func_ptr_is_kernel_text(void *ptr)
{
	unsigned long addr;
	addr = (unsigned long) dereference_function_descriptor(ptr);
	if (core_kernel_text(addr))
		return 1;
	return is_module_text_address(addr);
}
