


#ifndef __BFA_DEBUG_H__
#define __BFA_DEBUG_H__

#define bfa_assert(__cond)	do {					\
	if (!(__cond)) 							\
		bfa_panic(__LINE__, __FILE__, #__cond);      \
} while (0)

#define bfa_sm_fault(__mod, __event)	do {				\
	bfa_sm_panic((__mod)->logm, __LINE__, __FILE__, __event);      \
} while (0)

#ifndef BFA_PERF_BUILD
#define bfa_assert_fp(__cond)	bfa_assert(__cond)
#else
#define bfa_assert_fp(__cond)
#endif

struct bfa_log_mod_s;
void bfa_panic(int line, char *file, char *panicstr);
void bfa_sm_panic(struct bfa_log_mod_s *logm, int line, char *file, int event);

#endif /* __BFA_DEBUG_H__ */
