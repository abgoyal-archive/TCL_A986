
#ifndef __BFAD_PERF_H__
#define __BFAD_PERF_H__

#ifdef BFAD_PERF_BUILD

#undef bfa_trc
#undef bfa_trc32
#undef bfa_assert
#undef BFA_TRC_FILE

#define bfa_trc(_trcp, _data)
#define bfa_trc32(_trcp, _data)
#define bfa_assert(__cond)
#define BFA_TRC_FILE(__mod, __submod)

#endif

#endif /* __BFAD_PERF_H__ */
