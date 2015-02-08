




#undef HMM_ADDR_ALIGN_UNIT
#undef HMM_BLOCK_ALIGN_UNIT
#undef HMM_UNIQUE
#undef HMM_DESC_PARAM
#undef HMM_SYM_TO_STRING
#undef HMM_SYM_TO_STRING
#undef HMM_AUDIT_FAIL

#define HMM_SYM_TO_STRING(X) HMM_SYM_TO_STRING(X)
#define HMM_SYM_TO_STRING(X) #X

#ifndef HMM_CNFG_NUM

/* Default configuration. */

/* Use hmm_ prefix to avoid identifier conflicts. */
#define HMM_UNIQUE(BASE) hmm_ ## BASE

/* Number of bytes in an Address Alignment Unit (AAU). */
//fwg
//#define HMM_ADDR_ALIGN_UNIT sizeof(int)
#define HMM_ADDR_ALIGN_UNIT 32

/* Number of AAUs in a Block Alignment Unit (BAU). */
#define HMM_BLOCK_ALIGN_UNIT 1

/* Type of unsigned integer big enough to hold the size of a Block in AAUs. */
typedef unsigned long HMM_UNIQUE(size_aau);

typedef unsigned long HMM_UNIQUE(size_bau);

void hmm_dflt_abort(const char *, const char *);

#define HMM_AUDIT_FAIL \
    hmm_dflt_abort(__FILE__, HMM_SYM_TO_STRING(__LINE__));

#elif HMM_CNFG_NUM == 0

/* Definitions for testing. */

#define HMM_UNIQUE(BASE) thmm_ ## BASE

#define HMM_ADDR_ALIGN_UNIT sizeof(int)

#define HMM_BLOCK_ALIGN_UNIT 3

typedef unsigned HMM_UNIQUE(size_aau);

typedef unsigned short HMM_UNIQUE(size_bau);


extern jmp_buf HMM_UNIQUE(jmp_buf);
extern const char *HMM_UNIQUE(fail_file);
extern unsigned HMM_UNIQUE(fail_line);

#define HMM_AUDIT_FAIL \
    { HMM_UNIQUE(fail_file) = __FILE__; HMM_UNIQUE(fail_line) = __LINE__; \
        longjmp(HMM_UNIQUE(jmp_buf), 1); }

#elif HMM_CNFG_NUM == 1

/* Put configuration 1 definitions here (if there is a configuration 1). */

#elif HMM_CNFG_NUM == 2

/* Put configuration 2 definitions here. */

#elif HMM_CNFG_NUM == 3

/* Put configuration 3 definitions here. */

#elif HMM_CNFG_NUM == 4

/* Put configuration 4 definitions here. */

#elif HMM_CNFG_NUM == 5

/* Put configuration 5 definitions here. */

#endif
