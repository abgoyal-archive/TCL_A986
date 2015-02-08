


#ifndef QEMU_MEMCHECK_MEMCHECK_UTIL_H
#define QEMU_MEMCHECK_MEMCHECK_UTIL_H

#ifndef CONFIG_MEMCHECK
#error CONFIG_MEMCHECK is not defined.
#endif  // CONFIG_MEMCHECK

#include "memcheck_common.h"
#include "elff_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Transfering data between guest and emulator address spaces.
// =============================================================================

void memcheck_get_guest_buffer(void* qemu_address,
                               target_ulong guest_address,
                               size_t buffer_size);

void memcheck_set_guest_buffer(target_ulong guest_address,
                               const void* qemu_address,
                               size_t buffer_size);

size_t memcheck_get_guest_string(char* qemu_str,
                                 target_ulong guest_str,
                                 size_t qemu_buffer_size);

size_t memcheck_get_guest_kernel_string(char* qemu_str,
                                        target_ulong guest_str,
                                        size_t qemu_buffer_size);

// =============================================================================
// Helpers for transfering memory allocation information.
// =============================================================================

static inline void
memcheck_get_malloc_descriptor(MallocDesc* qemu_address,
                               target_ulong guest_address)
{
    memcheck_get_guest_buffer(qemu_address, guest_address, sizeof(MallocDesc));
}

static inline void
memcheck_set_malloc_descriptor(target_ulong guest_address,
                               const MallocDesc* qemu_address)
{
    memcheck_set_guest_buffer(guest_address, qemu_address, sizeof(MallocDesc));
}

static inline void
memcheck_get_free_descriptor(MallocFree* qemu_address,
                             target_ulong guest_address)
{
    memcheck_get_guest_buffer(qemu_address, guest_address, sizeof(MallocFree));
}

static inline void
memcheck_get_query_descriptor(MallocDescQuery* qemu_address,
                              target_ulong guest_address)
{
    memcheck_get_guest_buffer(qemu_address, guest_address,
                              sizeof(MallocDescQuery));
}

void memcheck_fail_alloc(target_ulong guest_address);

void memcheck_fail_free(target_ulong guest_address);

void memcheck_fail_query(target_ulong guest_address);

// =============================================================================
// Misc. utility routines.
// =============================================================================

static inline target_ulong
memcheck_tpc_to_gpc(target_ulong tb_pc)
{
    const TranslationBlock* tb = tb_find_pc(tb_pc);
    return tb != NULL ? tb_search_guest_pc_from_tb_pc(tb, tb_pc) : 0;
}

void invalidate_tlb_cache(target_ulong start, target_ulong end);

int memcheck_get_address_info(target_ulong abs_pc,
                              const MMRangeDesc* rdesc,
                              Elf_AddressInfo* info,
                              ELFF_HANDLE* elff_handle);

void memcheck_dump_malloc_desc(const MallocDescEx* desc,
                               int print_flags,
                               int print_proc_info);

#ifdef __cplusplus
};  /* end of extern "C" */
#endif

#endif  // QEMU_MEMCHECK_MEMCHECK_UTIL_H
