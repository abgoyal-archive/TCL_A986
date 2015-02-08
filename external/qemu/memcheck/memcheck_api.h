


#ifndef QEMU_MEMCHECK_MEMCHECK_API_H
#define QEMU_MEMCHECK_MEMCHECK_API_H

#ifndef CONFIG_MEMCHECK
#error CONFIG_MEMCHECK is not defined.
#endif  // CONFIG_MEMCHECK

extern int memcheck_enabled;

extern int memcheck_instrument_mmu;

extern int memcheck_watch_call_stack;

extern target_ulong* gen_opc_tpc2gpc_ptr;

extern unsigned int gen_opc_tpc2gpc_pairs;

int memcheck_is_checked(target_ulong addr, uint32_t size);

int memcheck_validate_ld(target_ulong addr,
                         uint32_t data_size,
                         target_ulong retaddr);

int memcheck_validate_st(target_ulong addr,
                         uint32_t data_size,
                         uint64_t value,
                         target_ulong retaddr);

void memcheck_on_call(target_ulong pc, target_ulong ret);

void memcheck_on_ret(target_ulong pc);

#endif  // QEMU_MEMCHECK_MEMCHECK_API_H
