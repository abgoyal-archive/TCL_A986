

#include <sys/sysctl.h>

int processIsCrashing(int pid)
{
    int mib[4] = { CTL_KERN, KERN_PROC, KERN_PROC_PID, pid };
    struct kinfo_proc info;
    size_t bufferSize = sizeof(info);
    if (sysctl(mib, sizeof(mib) / sizeof(mib[0]), &info, &bufferSize, 0, 0)) {
        perror("sysctl");
        return 0;
    }

    struct extern_proc proc = info.kp_proc;

    // The process is crashing if it is waiting to exit, is not a zombie, and has a non-zero exit code.
    return proc.p_stat != SZOMB && (proc.p_flag & P_WEXIT) && proc.p_xstat;
}
