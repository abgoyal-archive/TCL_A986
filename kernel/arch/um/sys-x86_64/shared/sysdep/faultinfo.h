

#ifndef __FAULTINFO_X86_64_H
#define __FAULTINFO_X86_64_H

struct faultinfo {
        int error_code; /* in ptrace_faultinfo misleadingly called is_write */
        unsigned long cr2; /* in ptrace_faultinfo called addr */
        int trap_no; /* missing in ptrace_faultinfo */
};

#define FAULT_WRITE(fi) ((fi).error_code & 2)
#define FAULT_ADDRESS(fi) ((fi).cr2)

#define PTRACE_FULL_FAULTINFO 1

#endif
