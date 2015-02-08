

#import <Foundation/Foundation.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t WebMemorySize(void);
unsigned long long WebVolumeFreeSize(NSString *path);
int WebNumberOfCPUs(void);

#ifdef __cplusplus
}
#endif
