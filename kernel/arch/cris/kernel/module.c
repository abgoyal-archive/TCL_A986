
#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#if 0
#define DEBUGP printk
#else
#define DEBUGP(fmt , ...)
#endif

#ifdef CONFIG_ETRAX_KMALLOCED_MODULES
#define MALLOC_MODULE(size) kmalloc(size, GFP_KERNEL)
#define FREE_MODULE(region) kfree(region)
#else
#define MALLOC_MODULE(size) vmalloc_exec(size)
#define FREE_MODULE(region) vfree(region)
#endif

void *module_alloc(unsigned long size)
{
	if (size == 0)
		return NULL;
	return MALLOC_MODULE(size);
}


/* Free memory returned from module_alloc */
void module_free(struct module *mod, void *module_region)
{
	FREE_MODULE(module_region);
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
	printk(KERN_ERR "module %s: REL relocation unsupported\n", me->name);
	return -ENOEXEC;
}

int apply_relocate_add(Elf32_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
  	unsigned int i;
	Elf32_Rela *rela = (void *)sechdrs[relsec].sh_addr;

	DEBUGP ("Applying add relocate section %u to %u\n", relsec,
		sechdrs[relsec].sh_info);

	for (i = 0; i < sechdrs[relsec].sh_size / sizeof (*rela); i++) {
		/* This is where to make the change */
		uint32_t *loc
			= ((void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			   + rela[i].r_offset);
		/* This is the symbol it is referring to.  Note that all
		   undefined symbols have been resolved.  */
		Elf32_Sym *sym
			= ((Elf32_Sym *)sechdrs[symindex].sh_addr
			   + ELF32_R_SYM (rela[i].r_info));
		switch (ELF32_R_TYPE(rela[i].r_info)) {
		case R_CRIS_32:
			*loc = sym->st_value + rela[i].r_addend;
			break;
		case R_CRIS_32_PCREL:
			*loc = sym->st_value - (unsigned)loc + rela[i].r_addend - 4;
			 break;
		default:
			printk(KERN_ERR "module %s: Unknown relocation: %u\n",
			       me->name, ELF32_R_TYPE(rela[i].r_info));
			return -ENOEXEC;
		}
	}

	return 0;
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
