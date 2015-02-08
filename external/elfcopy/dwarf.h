
#include "dwarf2.h"

typedef unsigned long dwarf_vma;
typedef unsigned long dwarf_size_type;

struct dwarf_section
{
  const char *name;
  unsigned char *start;
  dwarf_vma address;
  dwarf_size_type size;
};

struct dwarf_section_display
{
  struct dwarf_section section;
  int (*display) (struct dwarf_section *, void *);
  unsigned int relocate : 1;
  unsigned int eh_frame : 1;
};

enum dwarf_section_display_enum {
  abbrev = 0,
  aranges,
  frame,
  info,
  line,
  pubnames,
  eh_frame,
  macinfo,
  str,
  loc,
  pubtypes,
  ranges,
  static_func,
  static_vars,
  types,
  weaknames,
  max
};

extern struct dwarf_section_display debug_displays [];

typedef struct
{
  unsigned int   pointer_size;
  unsigned long  cu_offset;
  unsigned long	 base_address;
  /* This is an array of offsets to the location list table.  */
  unsigned long *loc_offsets;
  int		*have_frame_base;
  unsigned int   num_loc_offsets;
  unsigned int   max_loc_offsets;
#ifdef SORT_LOCATION_LIST_OFFSETS
  unsigned long  last_loc_offset; /* used to determine whether loc offsets are sorted */
#endif
  unsigned long *range_lists;
  unsigned int   num_range_lists;
  unsigned int   max_range_lists;
}
debug_info;

extern dwarf_vma (*byte_get) (unsigned char *, int);
extern dwarf_vma byte_get_little_endian (unsigned char *, int);
extern dwarf_vma byte_get_big_endian (unsigned char *, int);

extern dwarf_vma eh_addr_size;
extern int is_relocatable;

extern int do_debug_info;
extern int do_debug_abbrevs;
extern int do_debug_lines;
extern int do_debug_pubnames;
extern int do_debug_aranges;
extern int do_debug_ranges;
extern int do_debug_frames;
extern int do_debug_frames_interp;
extern int do_debug_macinfo;
extern int do_debug_str;
extern int do_debug_loc;

extern int load_debug_section (enum dwarf_section_display_enum,
			       void *);
extern void free_debug_section (enum dwarf_section_display_enum);

extern void free_debug_memory (void);

extern void base_value_pair_hook(void *data, int size,
                                 int base, int begin, int end);

extern void value_hook(void *data, int size, int val);

extern void signed_value_hook(void *data,
                              int pointer_size,
                              int is_signed,
                              int val);

extern void init_dwarf_variables(void);
