

#ifndef _ANDROID_UTILS_FILELOCK_H
#define _ANDROID_UTILS_FILELOCK_H


typedef struct FileLock  FileLock;

extern FileLock*  filelock_create ( const char*  path );
extern void       filelock_release( FileLock*  lock );

#endif /* _ANDROID_UTILS_FILELOCK_H */
