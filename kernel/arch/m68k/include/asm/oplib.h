

#ifndef __SPARC_OPLIB_H
#define __SPARC_OPLIB_H

#include <asm/openprom.h>

/* The master romvec pointer... */
extern struct linux_romvec *romvec;

/* Enumeration to describe the prom major version we have detected. */
enum prom_major_version {
	PROM_V0,      /* Original sun4c V0 prom */
	PROM_V2,      /* sun4c and early sun4m V2 prom */
	PROM_V3,      /* sun4m and later, up to sun4d/sun4e machines V3 */
	PROM_P1275,   /* IEEE compliant ISA based Sun PROM, only sun4u */
};

extern enum prom_major_version prom_vers;
/* Revision, and firmware revision. */
extern unsigned int prom_rev, prom_prev;

extern int prom_root_node;

extern struct linux_nodeops *prom_nodeops;

/* The functions... */

extern void prom_init(struct linux_romvec *rom_ptr);

/* Boot argument acquisition, returns the boot command line string. */
extern char *prom_getbootargs(void);

/* Device utilities. */

extern char *prom_mapio(char *virt_hint, int io_space, unsigned int phys_addr, unsigned int num_bytes);
extern void prom_unmapio(char *virt_addr, unsigned int num_bytes);

/* Device operations. */

extern int prom_devopen(char *device_string);

extern int prom_devclose(int device_handle);

extern void prom_seek(int device_handle, unsigned int seek_hival,
		      unsigned int seek_lowval);

/* Machine memory configuration routine. */

extern struct linux_mem_v0 *prom_meminfo(void);

/* Miscellaneous routines, don't really fit in any category per se. */

/* Reboot the machine with the command line passed. */
extern void prom_reboot(char *boot_command);

/* Evaluate the forth string passed. */
extern void prom_feval(char *forth_string);

extern void prom_cmdline(void);

extern void prom_halt(void);

typedef void (*sync_func_t)(void);
extern void prom_setsync(sync_func_t func_ptr);

extern unsigned char prom_get_idprom(char *idp_buffer, int idpbuf_size);

/* Get the prom major version. */
extern int prom_version(void);

/* Get the prom plugin revision. */
extern int prom_getrev(void);

/* Get the prom firmware revision. */
extern int prom_getprev(void);

/* Character operations to/from the console.... */

/* Non-blocking get character from console. */
extern int prom_nbgetchar(void);

/* Non-blocking put character to console. */
extern int prom_nbputchar(char character);

/* Blocking get character from console. */
extern char prom_getchar(void);

/* Blocking put character to console. */
extern void prom_putchar(char character);

/* Prom's internal printf routine, don't use in kernel/boot code. */
void prom_printf(char *fmt, ...);

/* Query for input device type */

enum prom_input_device {
	PROMDEV_IKBD,			/* input from keyboard */
	PROMDEV_ITTYA,			/* input from ttya */
	PROMDEV_ITTYB,			/* input from ttyb */
	PROMDEV_I_UNK,
};

extern enum prom_input_device prom_query_input_device(void);

/* Query for output device type */

enum prom_output_device {
	PROMDEV_OSCREEN,		/* to screen */
	PROMDEV_OTTYA,			/* to ttya */
	PROMDEV_OTTYB,			/* to ttyb */
	PROMDEV_O_UNK,
};

extern enum prom_output_device prom_query_output_device(void);

/* Multiprocessor operations... */

extern int prom_startcpu(int cpunode, struct linux_prom_registers *context_table,
			 int context, char *program_counter);

/* Stop the CPU with the passed device tree node. */
extern int prom_stopcpu(int cpunode);

/* Idle the CPU with the passed device tree node. */
extern int prom_idlecpu(int cpunode);

/* Re-Start the CPU with the passed device tree node. */
extern int prom_restartcpu(int cpunode);

/* PROM memory allocation facilities... */

extern char *prom_alloc(char *virt_hint, unsigned int size);

/* Free a previously allocated chunk. */
extern void prom_free(char *virt_addr, unsigned int size);

/* Sun4/sun4c specific memory-management startup hook. */

extern void prom_putsegment(int context, unsigned long virt_addr,
			    int physical_segment);

/* PROM device tree traversal functions... */

/* Get the child node of the given node, or zero if no child exists. */
extern int prom_getchild(int parent_node);

extern int prom_getsibling(int node);

extern int prom_getproplen(int thisnode, char *property);

extern int prom_getproperty(int thisnode, char *property,
			    char *prop_buffer, int propbuf_size);

/* Acquire an integer property. */
extern int prom_getint(int node, char *property);

/* Acquire an integer property, with a default value. */
extern int prom_getintdefault(int node, char *property, int defval);

/* Acquire a boolean property, 0=FALSE 1=TRUE. */
extern int prom_getbool(int node, char *prop);

/* Acquire a string property, null string on error. */
extern void prom_getstring(int node, char *prop, char *buf, int bufsize);

/* Does the passed node have the given "name"? YES=1 NO=0 */
extern int prom_nodematch(int thisnode, char *name);

extern int prom_searchsiblings(int node_start, char *name);

extern char *prom_firstprop(int node);

extern char *prom_nextprop(int node, char *prev_property);

/* Returns 1 if the specified node has given property. */
extern int prom_node_has_property(int node, char *property);

extern int prom_setprop(int node, char *prop_name, char *prop_value,
			int value_size);

extern int prom_pathtoinode(char *path);
extern int prom_inst2pkg(int);

/* Dorking with Bus ranges... */

/* Adjust reg values with the passed ranges. */
extern void prom_adjust_regs(struct linux_prom_registers *regp, int nregs,
			     struct linux_prom_ranges *rangep, int nranges);

/* Adjust child ranges with the passed parent ranges. */
extern void prom_adjust_ranges(struct linux_prom_ranges *cranges, int ncranges,
			       struct linux_prom_ranges *pranges, int npranges);

/* Apply promlib probed OBIO ranges to registers. */
extern void prom_apply_obio_ranges(struct linux_prom_registers *obioregs, int nregs);

/* Apply ranges of any prom node (and optionally parent node as well) to registers. */
extern void prom_apply_generic_ranges(int node, int parent,
				      struct linux_prom_registers *sbusregs, int nregs);


#endif /* !(__SPARC_OPLIB_H) */
