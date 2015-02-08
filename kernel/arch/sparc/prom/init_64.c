

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/ctype.h>

#include <asm/openprom.h>
#include <asm/oplib.h>

/* OBP version string. */
char prom_version[80];

/* The root node of the prom device tree. */
int prom_stdin, prom_stdout;
int prom_chosen_node;


extern void prom_cif_init(void *, void *);

void __init prom_init(void *cif_handler, void *cif_stack)
{
	int node;

	prom_cif_init(cif_handler, cif_stack);

	prom_chosen_node = prom_finddevice(prom_chosen_path);
	if (!prom_chosen_node || prom_chosen_node == -1)
		prom_halt();

	prom_stdin = prom_getint(prom_chosen_node, "stdin");
	prom_stdout = prom_getint(prom_chosen_node, "stdout");

	node = prom_finddevice("/openprom");
	if (!node || node == -1)
		prom_halt();

	prom_getstring(node, "version", prom_version, sizeof(prom_version));

	prom_printf("\n");
}

void __init prom_init_report(void)
{
	printk("PROMLIB: Sun IEEE Boot Prom '%s'\n", prom_version);
	printk("PROMLIB: Root node compatible: %s\n", prom_root_compatible);
}
