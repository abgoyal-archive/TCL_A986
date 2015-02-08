
#include <linux/sched.h>

struct mt_proc_struct{
    int pid;
    int tgid;
    int index;
    u64 cputime;
    u64 cputime_init;
    u64 prof_start;
    u64 prof_end;

    cputime_t utime_init;
    cputime_t utime;
    cputime_t stime_init;
    cputime_t stime;
    char comm[TASK_COMM_LEN];
};
#define PROC_NUM 512
