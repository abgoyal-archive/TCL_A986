
#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>

#if 0
#define DEBUGP printk
#else
#define DEBUGP(fmt...)
#endif

void *module_alloc(unsigned long size)
{
	if (size == 0)
		return NULL;

	return vmalloc_exec(size);
}


/* Free memory returned from module_alloc */
void module_free(struct module *mod, void *module_region)
{
	vfree(module_region);
}

/* We don't need anything special. */
int module_frob_arch_sections(Elf_Ehdr *hdr,
			      Elf_Shdr *sechdrs,
			      char *secstrings,
			      struct module *mod)
{
	return 0;
}

int apply_relocate(Elf32_Shdr *sechdrs,
		   const char *strtab,
		   unsigned int symindex,
		   unsigned int relsec,
		   struct module *me)
{
	printk(KERN_ERR "module %s: ADD RELOCATION unsupported\n", me->name);
	return -ENOEXEC;
}

int apply_relocate_add(Elf32_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
	printk(KERN_ERR "module %s: ADD RELOCATION unsupported\n", me->name);
	return -ENOEXEC;
}

int module_finalize(const Elf_Ehdr *hdr,
		    const Elf_Shdr *sechdrs,
		    struct module *me)
{
	return 0;
}

void module_arch_cleanup(struct module *mod)
{
}
