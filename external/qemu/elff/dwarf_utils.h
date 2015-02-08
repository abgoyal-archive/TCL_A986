


#ifndef ELFF_DWARF_UTILS_
#define ELFF_DWARF_UTILS_

#include "dwarf_defs.h"

const char* dwarf_at_name(Dwarf_At at);

const char* dwarf_form_name(Dwarf_Form form);

const char* dwarf_tag_name(Dwarf_Tag tag);

void dump_attrib(Dwarf_At at, Dwarf_Form form, const Dwarf_Value* val);

void dump_value(const Dwarf_Value* val);

#endif  // ELFF_DWARF_UTILS_
