
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#ifdef CONFIG_SOLARIS
#include <sys/types.h>
#include <sys/statvfs.h>
#endif

/* Needed early for CONFIG_BSD etc. */
#include "config-host.h"

#ifdef _WIN32
#include <windows.h>
#elif defined(CONFIG_BSD)
#include <stdlib.h>
#else
#include <malloc.h>
#endif

#include "qemu-common.h"
#include "sysemu.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(CONFIG_BSD)
#include <stdlib.h>
#else
#include <malloc.h>
#endif


#if defined(_WIN32)
void *qemu_memalign(size_t alignment, size_t size)
{
    return VirtualAlloc(NULL, size, MEM_COMMIT, PAGE_READWRITE);
}

void *qemu_vmalloc(size_t size)
{
    /* FIXME: this is not exactly optimal solution since VirtualAlloc
       has 64Kb granularity, but at least it guarantees us that the
       memory is page aligned. */
    return VirtualAlloc(NULL, size, MEM_COMMIT, PAGE_READWRITE);
}

void qemu_vfree(void *ptr)
{
    VirtualFree(ptr, 0, MEM_RELEASE);
}

#else

#if defined(CONFIG_KQEMU)

#ifdef __OpenBSD__
#include <sys/param.h>
#include <sys/types.h>
#include <sys/mount.h>
#else
#ifndef __FreeBSD__
#include <sys/vfs.h>
#endif
#endif

#include <sys/mman.h>
#include <fcntl.h>

static void *kqemu_vmalloc(size_t size)
{
    static int phys_ram_fd = -1;
    static int phys_ram_size = 0;
    void *ptr;

/* no need (?) for a dummy file on OpenBSD/FreeBSD */
#if defined(__OpenBSD__) || defined(__FreeBSD__) || defined(__DragonFly__)
    int map_anon = MAP_ANON;
#else
    int map_anon = 0;
    const char *tmpdir;
    char phys_ram_file[1024];
#ifdef CONFIG_SOLARIS
    struct statvfs stfs;
#else
    struct statfs stfs;
#endif

    if (phys_ram_fd < 0) {
        tmpdir = getenv("QEMU_TMPDIR");
        if (!tmpdir)
#ifdef CONFIG_SOLARIS
            tmpdir = "/tmp";
        if (statvfs(tmpdir, &stfs) == 0) {
#else
            tmpdir = "/dev/shm";
        if (statfs(tmpdir, &stfs) == 0) {
#endif
            int64_t free_space;
            int ram_mb;

            free_space = (int64_t)stfs.f_bavail * stfs.f_bsize;
            if ((ram_size + 8192 * 1024) >= free_space) {
                ram_mb = (ram_size / (1024 * 1024));
                fprintf(stderr,
                        "You do not have enough space in '%s' for the %d MB of QEMU virtual RAM.\n",
                        tmpdir, ram_mb);
                if (strcmp(tmpdir, "/dev/shm") == 0) {
                    fprintf(stderr, "To have more space available provided you have enough RAM and swap, do as root:\n"
                            "mount -o remount,size=%dm /dev/shm\n",
                            ram_mb + 16);
                } else {
                    fprintf(stderr,
                            "Use the '-m' option of QEMU to diminish the amount of virtual RAM or use the\n"
                            "QEMU_TMPDIR environment variable to set another directory where the QEMU\n"
                            "temporary RAM file will be opened.\n");
                }
                fprintf(stderr, "Or disable the accelerator module with -no-kqemu\n");
                exit(1);
            }
        }
        snprintf(phys_ram_file, sizeof(phys_ram_file), "%s/qemuXXXXXX",
                 tmpdir);
        phys_ram_fd = mkstemp(phys_ram_file);
        if (phys_ram_fd < 0) {
            fprintf(stderr,
                    "warning: could not create temporary file in '%s'.\n"
                    "Use QEMU_TMPDIR to select a directory in a tmpfs filesystem.\n"
                    "Using '/tmp' as fallback.\n",
                    tmpdir);
            snprintf(phys_ram_file, sizeof(phys_ram_file), "%s/qemuXXXXXX",
                     "/tmp");
            phys_ram_fd = mkstemp(phys_ram_file);
            if (phys_ram_fd < 0) {
                fprintf(stderr, "Could not create temporary memory file '%s'\n",
                        phys_ram_file);
                exit(1);
            }
        }
        unlink(phys_ram_file);
    }
    size = (size + 4095) & ~4095;
    ftruncate(phys_ram_fd, phys_ram_size + size);
#endif /* !(__OpenBSD__ || __FreeBSD__ || __DragonFly__) */
    ptr = mmap(NULL,
               size,
               PROT_WRITE | PROT_READ, map_anon | MAP_SHARED,
               phys_ram_fd, phys_ram_size);
    if (ptr == MAP_FAILED) {
        fprintf(stderr, "Could not map physical memory\n");
        exit(1);
    }
    phys_ram_size += size;
    return ptr;
}

static void kqemu_vfree(void *ptr)
{
    /* may be useful some day, but currently we do not need to free */
}

#endif

void *qemu_memalign(size_t alignment, size_t size)
{
#if defined(_POSIX_C_SOURCE)
    int ret;
    void *ptr;
    ret = posix_memalign(&ptr, alignment, size);
    if (ret != 0)
        return NULL;
    return ptr;
#elif defined(CONFIG_BSD)
    return valloc(size);
#else
    return memalign(alignment, size);
#endif
}

/* alloc shared memory pages */
void *qemu_vmalloc(size_t size)
{
#if defined(CONFIG_KQEMU)
    if (kqemu_allowed)
        return kqemu_vmalloc(size);
#endif
    return qemu_memalign(getpagesize(), size);
}

void qemu_vfree(void *ptr)
{
#if defined(CONFIG_KQEMU)
    if (kqemu_allowed)
        kqemu_vfree(ptr);
#endif
    free(ptr);
}

#endif

int qemu_create_pidfile(const char *filename)
{
    char buffer[128];
    int len;
#ifndef _WIN32
    int fd;

    fd = open(filename, O_RDWR | O_CREAT, 0600);
    if (fd == -1)
        return -1;

    if (lockf(fd, F_TLOCK, 0) == -1)
        return -1;

    len = snprintf(buffer, sizeof(buffer), "%ld\n", (long)getpid());
    if (write(fd, buffer, len) != len)
        return -1;
#else
    HANDLE file;
    DWORD flags;
    OVERLAPPED overlap;
    BOOL ret;

    /* Open for writing with no sharing. */
    file = CreateFile(filename, GENERIC_WRITE, 0, NULL,
		      OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    if (file == INVALID_HANDLE_VALUE)
      return -1;

    flags = LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY;
    overlap.hEvent = 0;
    /* Lock 1 byte. */
    ret = LockFileEx(file, flags, 0, 0, 1, &overlap);
    if (ret == 0)
      return -1;

    /* Write PID to file. */
    len = snprintf(buffer, sizeof(buffer), "%ld\n", (long)getpid());
    ret = WriteFileEx(file, (LPCVOID)buffer, (DWORD)len,
		      &overlap, NULL);
    if (ret == 0)
      return -1;
#endif
    return 0;
}

#ifdef _WIN32

/* Offset between 1/1/1601 and 1/1/1970 in 100 nanosec units */
#define _W32_FT_OFFSET (116444736000000000ULL)

int qemu_gettimeofday(qemu_timeval *tp)
{
  union {
    unsigned long long ns100; /*time since 1 Jan 1601 in 100ns units */
    FILETIME ft;
  }  _now;

  if(tp)
    {
      GetSystemTimeAsFileTime (&_now.ft);
      tp->tv_usec=(long)((_now.ns100 / 10ULL) % 1000000ULL );
      tp->tv_sec= (long)((_now.ns100 - _W32_FT_OFFSET) / 10000000ULL);
    }
  /* Always return 0 as per Open Group Base Specifications Issue 6.
     Do not set errno on error.  */
  return 0;
}

int  ffs(int  val)
{
	int  nn;
	
	for (nn = 0; nn < sizeof(int)*8; nn++)
		if (val & (1 << nn))
			return nn+1;

	return 0;
}

#endif /* _WIN32 */


