

#import <Foundation/Foundation.h>

#define WEB_UREAD     (00400)   /* Read by owner */
#define WEB_UWRITE    (00200)   /* Write by owner */
#define WEB_UEXEC     (00100)   /* Execute/Search by owner */

@interface NSFileManager (WebNSFileManagerExtras)

- (void)_webkit_backgroundRemoveFileAtPath:(NSString *)path;
- (void)_webkit_backgroundRemoveLeftoverFiles:(NSString *)path;
- (BOOL)_webkit_removeFileOnlyAtPath:(NSString *)path;
- (void)_webkit_setMetadataURL:(NSString *)URLString referrer:(NSString *)referrer atPath:(NSString *)path;
- (NSString *)_webkit_startupVolumeName;
- (NSString *)_webkit_pathWithUniqueFilenameForPath:(NSString *)path;

@end


#ifdef BUILDING_ON_TIGER
@interface NSFileManager (WebNSFileManagerTigerForwardCompatibility)
- (NSArray *)contentsOfDirectoryAtPath:(NSString *)path error:(NSError **)error;
- (BOOL)moveItemAtPath:(NSString *)srcPath toPath:(NSString *)dstPath error:(NSError **)error;
- (BOOL)removeItemAtPath:(NSString *)path error:(NSError **)error;
- (NSDictionary *)attributesOfFileSystemForPath:(NSString *)path error:(NSError **)error;
- (NSString *)destinationOfSymbolicLinkAtPath:(NSString *)path error:(NSError **)error;
- (NSDictionary *)attributesOfItemAtPath:(NSString *)path error:(NSError **)error;
@end
#endif
