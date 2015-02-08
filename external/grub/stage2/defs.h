

#define	DEV_BSIZE	512

#define	btodb(byte_offset)	((byte_offset) >> 9)
#define	dbtob(block_number)	((block_number) << 9)


typedef unsigned char u_char;	/* unsigned char */
typedef unsigned short u_short;	/* unsigned short */
typedef unsigned int u_int;	/* unsigned int */

typedef struct _quad_
  {
    unsigned int val[2];	/* 2 int values make... */
  }
quad;				/* an 8-byte item */

typedef unsigned int mach_time_t;	/* an unsigned int */
typedef unsigned int mach_daddr_t;	/* an unsigned int */
typedef unsigned int mach_off_t;	/* another unsigned int */

typedef unsigned short mach_uid_t;
typedef unsigned short mach_gid_t;
typedef unsigned int mach_ino_t;

#define	NBBY	8

#define	MAXBSIZE	8192
#define	MAXFRAG		8


#define	MAXPATHLEN	1024
#define	MAXSYMLINKS	8
