
#ifndef _UDF_FS_I_H
#define _UDF_FS_I_H 1

/* exported IOCTLs, we have 'l', 0x40-0x7f */
#define UDF_GETEASIZE   _IOR('l', 0x40, int)
#define UDF_GETEABLOCK  _IOR('l', 0x41, void *)
#define UDF_GETVOLIDENT _IOR('l', 0x42, void *)
#define UDF_RELOCATE_BLOCKS _IOWR('l', 0x43, long)

#endif /* _UDF_FS_I_H */
