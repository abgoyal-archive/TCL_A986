

#ifndef _ANDROID_UTILS_TEMPFILE_H
#define _ANDROID_UTILS_TEMPFILE_H


typedef struct TempFile   TempFile;

extern  TempFile*    tempfile_create( void );
extern  const char*  tempfile_path( TempFile*  temp );
extern  void         tempfile_close( TempFile*  temp );

extern void          atexit_close_fd(int  fd);
extern void          atexit_close_fd_remove(int  fd);

#endif /* _ANDROID_UTILS_TEMPFILE_H */
