

#ifndef _BOOT_UFS_DIR_H_
#define	_BOOT_UFS_DIR_H_

#define DIRBLKSIZ	DEV_BSIZE
#define	MAXNAMLEN	255

struct direct
  {
    u_int d_ino;		/* inode number of entry */
    u_short d_reclen;		/* length of this record */
    u_short d_namlen;		/* length of string in d_name */
    char d_name[MAXNAMLEN + 1];	/* name with length <= MAXNAMLEN */
  };

#undef DIRSIZ
#define DIRSIZ(dp) \
    ((sizeof (struct direct) - (MAXNAMLEN+1)) + (((dp)->d_namlen+1 + 3) &~ 3))

#ifdef KERNEL
struct dirtemplate
  {
    u_int dot_ino;
    short dot_reclen;
    short dot_namlen;
    char dot_name[4];		/* must be multiple of 4 */
    u_int dotdot_ino;
    short dotdot_reclen;
    short dotdot_namlen;
    char dotdot_name[4];	/* ditto */
  };
#endif

#ifndef KERNEL
#define d_fileno d_ino		/* compatibility with POSIX */
#ifndef DEV_BSIZE
#define	DEV_BSIZE	512
#endif
typedef struct _dirdesc
  {
    int dd_fd;
    int dd_loc;
    int dd_size;
    char dd_buf[DIRBLKSIZ];
  }
DIR;

#define dirfd(dirp)	((dirp)->dd_fd)

#ifndef NULL
#define NULL 0
#endif
extern DIR *opendir ();
extern struct direct *readdir ();
extern int telldir ();
extern void seekdir ();
#define rewinddir(dirp)	seekdir((dirp), (long)0)
extern void closedir ();
#endif /* not KERNEL */
#endif /* _BOOT_UFS_DIR_H_ */
