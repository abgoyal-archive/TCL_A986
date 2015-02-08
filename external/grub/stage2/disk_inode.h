
#ifndef	_BOOT_UFS_DISK_INODE_H_
#define	_BOOT_UFS_DISK_INODE_H_


#define	FFS_NDADDR	12	/* direct addresses in inode */
#define	FFS_NIADDR	3	/* indirect addresses in inode */

#define	FFS_MAX_FASTLINK_SIZE	((FFS_NDADDR + FFS_NIADDR) \
				 * sizeof (mach_daddr_t))

struct icommon
  {
    u_short ic_mode;		/*  0: mode and type of file */
    short ic_nlink;		/*  2: number of links to file */
    mach_uid_t ic_uid;		/*  4: owner's user id */
    mach_gid_t ic_gid;		/*  6: owner's group id */
    quad ic_size;		/*  8: number of bytes in file */
    mach_time_t ic_atime;	/* 16: time last accessed */
    int ic_atspare;
    mach_time_t ic_mtime;	/* 24: time last modified */
    int ic_mtspare;
    mach_time_t ic_ctime;	/* 32: last time inode changed */
    int ic_ctspare;
    union
      {
	struct
	  {
	    mach_daddr_t Mb_db[FFS_NDADDR];	/* 40: disk block addresses */
	    mach_daddr_t Mb_ib[FFS_NIADDR];	/* 88: indirect blocks */
	  }
	ic_Mb;
	char ic_Msymlink[FFS_MAX_FASTLINK_SIZE];
	/* 40: symbolic link name */
      }
    ic_Mun;
#define	ic_db		ic_Mun.ic_Mb.Mb_db
#define	ic_ib		ic_Mun.ic_Mb.Mb_ib
#define	ic_symlink	ic_Mun.ic_Msymlink
    int ic_flags;		/* 100: status, currently unused */
    int ic_blocks;		/* 104: blocks actually held */
    int ic_gen;			/* 108: generation number */
    int ic_spare[4];		/* 112: reserved, currently unused */
  };

struct dinode
  {
    union
      {
	struct icommon di_com;
	char di_char[128];
      }
    di_un;
  };
#define	di_ic	di_un.di_com

#endif /* _BOOT_UFS_DISK_INODE_H_ */
