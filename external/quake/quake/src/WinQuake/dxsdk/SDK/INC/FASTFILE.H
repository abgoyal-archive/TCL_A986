
typedef LPVOID  HFASTFILE;

extern BOOL FastFileInit( LPSTR fname, int max_handles );
extern void FastFileFini( void );
extern HFASTFILE FastFileOpen( LPSTR name );
extern BOOL FastFileClose( HFASTFILE pfe );
extern BOOL FastFileRead( HFASTFILE pfh, LPVOID ptr, int size );
extern BOOL FastFileSeek( HFASTFILE pfe, int off, int how );
extern long FastFileTell( HFASTFILE pfe );
extern LPVOID FastFileLock( HFASTFILE pfe, int off, int len );
extern BOOL FastFileUnlock( HFASTFILE pfe, int off, int len );
