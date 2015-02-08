


#ifndef ELFF_API_H_
#define ELFF_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "qemu-common.h"

/* Defines type for a handle used in ELFF API. */
typedef void* ELFF_HANDLE;

typedef struct Elf_InlineInfo {
  /* Name of the routine where previous routine is inlined.
   * This field can never be NULL, except for the last array entry.
   */
  const char*     routine_name;

  /* Source file name where routine is inlined.
   * This field can be NULL, if it was not possible to obtain information
   * about source file location for the routine. If this field is NULL, content
   * of inlined_in_file_dir and inlined_at_line fields is undefined and should
   * be ignored. */
  const char*     inlined_in_file;

  /* Source file directory where routine is inlined.
   * If inlined_in_file field contains NULL, content of this field is undefined
   * and should be ignored. */
  const char*     inlined_in_file_dir;

  /* Source file line number where routine is inlined.
   * If inlined_in_file field contains NULL, content of this field is undefined
   * and should be ignored. */
  uint32_t        inlined_at_line;
} Elf_InlineInfo;

static inline int
elfinlineinfo_is_last_entry(const Elf_InlineInfo* info) {
    return info->routine_name == 0;
}

typedef struct Elf_AddressInfo {
  /* Name of the routine containing the address. If name of the routine
   * was not available (DW_AT_name attribute was missing) this field
   * is set to "<unknown>". */
  const char*       routine_name;

  /* Name of the source file containing the routine. If source location for the
   * routine was not available, this field is set to NULL, and content of
   * dir_name, and line_number fields of this structure is not defined. */
  const char*       file_name;

  /* Path to the source file directory. If file_name field of this structure is
   * NULL, content of this field is not defined. */
  const char*       dir_name;

  /* Line number in the source file for the address. If file_name field of this
   * structure is NULL, content of this field is not defined. */
  uint32_t          line_number;

  /* If routine that contains the given address has been inlined (or it is part
   * of even deeper inline branch) this array lists information about that
   * inline branch rooting to the first routine that has not been inlined. The
   * first element in the array references a routine, where routine containing
   * the given address has been inlined. The second entry contains information
   * about a routine referenced by the first entry (and so on). If routine,
   * containing the given address has not been inlined, this field is set to
   * NULL. The array ends with an entry containing all zeroes. */
  Elf_InlineInfo*   inline_stack;
} Elf_AddressInfo;

//=============================================================================
// API routines
//=============================================================================

ELFF_HANDLE elff_init(const char* elf_file_path);

void elff_close(ELFF_HANDLE handle);

int elff_is_exec(ELFF_HANDLE handle);

int elff_get_pc_address_info(ELFF_HANDLE handle,
                             uint64_t address,
                             Elf_AddressInfo* address_info);

void elff_free_pc_address_info(ELFF_HANDLE handle,
                               Elf_AddressInfo* address_info);

#ifdef __cplusplus
}   /* end of extern "C" */
#endif

#endif  // ELFF_API_H_
