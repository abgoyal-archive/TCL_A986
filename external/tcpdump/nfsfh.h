
/* @(#) $Header: /tcpdump/master/tcpdump/nfsfh.h,v 1.13 2002/04/24 06:27:05 guy Exp $ (LBL) */



typedef struct {
	u_int32_t Minor;	/* upper case to avoid clashing with macro names */
	u_int32_t Major;
} my_devt;

#define	dev_eq(a,b)	((a.Minor == b.Minor) && (a.Major == b.Major))

typedef	struct {
	my_devt	Fsid_dev;		/* XXX avoid name conflict with AIX */
	char Opaque_Handle[2 * 32 + 1];
	u_int32_t fsid_code;
} my_fsid;

#define	fsid_eq(a,b)	((a.fsid_code == b.fsid_code) &&\
			 dev_eq(a.Fsid_dev, b.Fsid_dev))

extern void Parse_fh(const unsigned char *, int, my_fsid *, ino_t *, const char **, const char **, int);
