
#ifndef _LIBFDT_H
#define _LIBFDT_H

#include <libfdt_env.h>
#include <fdt.h>

#define FDT_FIRST_SUPPORTED_VERSION	0x10
#define FDT_LAST_SUPPORTED_VERSION	0x11

/* Error codes: informative error codes */
#define FDT_ERR_NOTFOUND	1
	/* FDT_ERR_NOTFOUND: The requested node or property does not exist */
#define FDT_ERR_EXISTS		2
	/* FDT_ERR_EXISTS: Attemped to create a node or property which
	 * already exists */
#define FDT_ERR_NOSPACE		3
	/* FDT_ERR_NOSPACE: Operation needed to expand the device
	 * tree, but its buffer did not have sufficient space to
	 * contain the expanded tree. Use fdt_open_into() to move the
	 * device tree to a buffer with more space. */

/* Error codes: codes for bad parameters */
#define FDT_ERR_BADOFFSET	4
	/* FDT_ERR_BADOFFSET: Function was passed a structure block
	 * offset which is out-of-bounds, or which points to an
	 * unsuitable part of the structure for the operation. */
#define FDT_ERR_BADPATH		5
	/* FDT_ERR_BADPATH: Function was passed a badly formatted path
	 * (e.g. missing a leading / for a function which requires an
	 * absolute path) */
#define FDT_ERR_BADPHANDLE	6
	/* FDT_ERR_BADPHANDLE: Function was passed an invalid phandle
	 * value.  phandle values of 0 and -1 are not permitted. */
#define FDT_ERR_BADSTATE	7
	/* FDT_ERR_BADSTATE: Function was passed an incomplete device
	 * tree created by the sequential-write functions, which is
	 * not sufficiently complete for the requested operation. */

/* Error codes: codes for bad device tree blobs */
#define FDT_ERR_TRUNCATED	8
	/* FDT_ERR_TRUNCATED: Structure block of the given device tree
	 * ends without an FDT_END tag. */
#define FDT_ERR_BADMAGIC	9
	/* FDT_ERR_BADMAGIC: Given "device tree" appears not to be a
	 * device tree at all - it is missing the flattened device
	 * tree magic number. */
#define FDT_ERR_BADVERSION	10
	/* FDT_ERR_BADVERSION: Given device tree has a version which
	 * can't be handled by the requested operation.  For
	 * read-write functions, this may mean that fdt_open_into() is
	 * required to convert the tree to the expected version. */
#define FDT_ERR_BADSTRUCTURE	11
	/* FDT_ERR_BADSTRUCTURE: Given device tree has a corrupt
	 * structure block or other serious error (e.g. misnested
	 * nodes, or subnodes preceding properties). */
#define FDT_ERR_BADLAYOUT	12
	/* FDT_ERR_BADLAYOUT: For read-write functions, the given
	 * device tree has it's sub-blocks in an order that the
	 * function can't handle (memory reserve map, then structure,
	 * then strings).  Use fdt_open_into() to reorganize the tree
	 * into a form suitable for the read-write operations. */

/* "Can't happen" error indicating a bug in libfdt */
#define FDT_ERR_INTERNAL	13
	/* FDT_ERR_INTERNAL: libfdt has failed an internal assertion.
	 * Should never be returned, if it is, it indicates a bug in
	 * libfdt itself. */

#define FDT_ERR_MAX		13

/**********************************************************************/
/* Low-level functions (you probably don't need these)                */
/**********************************************************************/

const void *fdt_offset_ptr(const void *fdt, int offset, int checklen);
static inline void *fdt_offset_ptr_w(void *fdt, int offset, int checklen)
{
	return (void *)(uintptr_t)fdt_offset_ptr(fdt, offset, checklen);
}

uint32_t fdt_next_tag(const void *fdt, int offset, int *nextoffset);

/**********************************************************************/
/* Traversal functions                                                */
/**********************************************************************/

int fdt_next_node(const void *fdt, int offset, int *depth);

/**********************************************************************/
/* General functions                                                  */
/**********************************************************************/

#define fdt_get_header(fdt, field) \
	(fdt32_to_cpu(((const struct fdt_header *)(fdt))->field))
#define fdt_magic(fdt) 			(fdt_get_header(fdt, magic))
#define fdt_totalsize(fdt)		(fdt_get_header(fdt, totalsize))
#define fdt_off_dt_struct(fdt)		(fdt_get_header(fdt, off_dt_struct))
#define fdt_off_dt_strings(fdt)		(fdt_get_header(fdt, off_dt_strings))
#define fdt_off_mem_rsvmap(fdt)		(fdt_get_header(fdt, off_mem_rsvmap))
#define fdt_version(fdt)		(fdt_get_header(fdt, version))
#define fdt_last_comp_version(fdt) 	(fdt_get_header(fdt, last_comp_version))
#define fdt_boot_cpuid_phys(fdt) 	(fdt_get_header(fdt, boot_cpuid_phys))
#define fdt_size_dt_strings(fdt) 	(fdt_get_header(fdt, size_dt_strings))
#define fdt_size_dt_struct(fdt)		(fdt_get_header(fdt, size_dt_struct))

#define __fdt_set_hdr(name) \
	static inline void fdt_set_##name(void *fdt, uint32_t val) \
	{ \
		struct fdt_header *fdth = fdt; \
		fdth->name = cpu_to_fdt32(val); \
	}
__fdt_set_hdr(magic);
__fdt_set_hdr(totalsize);
__fdt_set_hdr(off_dt_struct);
__fdt_set_hdr(off_dt_strings);
__fdt_set_hdr(off_mem_rsvmap);
__fdt_set_hdr(version);
__fdt_set_hdr(last_comp_version);
__fdt_set_hdr(boot_cpuid_phys);
__fdt_set_hdr(size_dt_strings);
__fdt_set_hdr(size_dt_struct);
#undef __fdt_set_hdr

int fdt_check_header(const void *fdt);

int fdt_move(const void *fdt, void *buf, int bufsize);

/**********************************************************************/
/* Read-only functions                                                */
/**********************************************************************/

const char *fdt_string(const void *fdt, int stroffset);

int fdt_num_mem_rsv(const void *fdt);

int fdt_get_mem_rsv(const void *fdt, int n, uint64_t *address, uint64_t *size);

int fdt_subnode_offset_namelen(const void *fdt, int parentoffset,
			       const char *name, int namelen);
int fdt_subnode_offset(const void *fdt, int parentoffset, const char *name);

int fdt_path_offset(const void *fdt, const char *path);

const char *fdt_get_name(const void *fdt, int nodeoffset, int *lenp);

const struct fdt_property *fdt_get_property(const void *fdt, int nodeoffset,
					    const char *name, int *lenp);
static inline struct fdt_property *fdt_get_property_w(void *fdt, int nodeoffset,
						      const char *name,
						      int *lenp)
{
	return (struct fdt_property *)(uintptr_t)
		fdt_get_property(fdt, nodeoffset, name, lenp);
}

const void *fdt_getprop(const void *fdt, int nodeoffset,
			const char *name, int *lenp);
static inline void *fdt_getprop_w(void *fdt, int nodeoffset,
				  const char *name, int *lenp)
{
	return (void *)(uintptr_t)fdt_getprop(fdt, nodeoffset, name, lenp);
}

uint32_t fdt_get_phandle(const void *fdt, int nodeoffset);

int fdt_get_path(const void *fdt, int nodeoffset, char *buf, int buflen);

int fdt_supernode_atdepth_offset(const void *fdt, int nodeoffset,
				 int supernodedepth, int *nodedepth);

int fdt_node_depth(const void *fdt, int nodeoffset);

int fdt_parent_offset(const void *fdt, int nodeoffset);

int fdt_node_offset_by_prop_value(const void *fdt, int startoffset,
				  const char *propname,
				  const void *propval, int proplen);

int fdt_node_offset_by_phandle(const void *fdt, uint32_t phandle);

int fdt_node_check_compatible(const void *fdt, int nodeoffset,
			      const char *compatible);

int fdt_node_offset_by_compatible(const void *fdt, int startoffset,
				  const char *compatible);

/**********************************************************************/
/* Write-in-place functions                                           */
/**********************************************************************/

int fdt_setprop_inplace(void *fdt, int nodeoffset, const char *name,
			const void *val, int len);

static inline int fdt_setprop_inplace_cell(void *fdt, int nodeoffset,
					   const char *name, uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_setprop_inplace(fdt, nodeoffset, name, &val, sizeof(val));
}

int fdt_nop_property(void *fdt, int nodeoffset, const char *name);

int fdt_nop_node(void *fdt, int nodeoffset);

/**********************************************************************/
/* Sequential write functions                                         */
/**********************************************************************/

int fdt_create(void *buf, int bufsize);
int fdt_add_reservemap_entry(void *fdt, uint64_t addr, uint64_t size);
int fdt_finish_reservemap(void *fdt);
int fdt_begin_node(void *fdt, const char *name);
int fdt_property(void *fdt, const char *name, const void *val, int len);
static inline int fdt_property_cell(void *fdt, const char *name, uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_property(fdt, name, &val, sizeof(val));
}
#define fdt_property_string(fdt, name, str) \
	fdt_property(fdt, name, str, strlen(str)+1)
int fdt_end_node(void *fdt);
int fdt_finish(void *fdt);

/**********************************************************************/
/* Read-write functions                                               */
/**********************************************************************/

int fdt_open_into(const void *fdt, void *buf, int bufsize);
int fdt_pack(void *fdt);

int fdt_add_mem_rsv(void *fdt, uint64_t address, uint64_t size);

int fdt_del_mem_rsv(void *fdt, int n);

int fdt_set_name(void *fdt, int nodeoffset, const char *name);

int fdt_setprop(void *fdt, int nodeoffset, const char *name,
		const void *val, int len);

static inline int fdt_setprop_cell(void *fdt, int nodeoffset, const char *name,
				   uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_setprop(fdt, nodeoffset, name, &val, sizeof(val));
}

#define fdt_setprop_string(fdt, nodeoffset, name, str) \
	fdt_setprop((fdt), (nodeoffset), (name), (str), strlen(str)+1)

int fdt_delprop(void *fdt, int nodeoffset, const char *name);

int fdt_add_subnode_namelen(void *fdt, int parentoffset,
			    const char *name, int namelen);

int fdt_add_subnode(void *fdt, int parentoffset, const char *name);

int fdt_del_node(void *fdt, int nodeoffset);

/**********************************************************************/
/* Debugging / informational functions                                */
/**********************************************************************/

const char *fdt_strerror(int errval);

#endif /* _LIBFDT_H */
