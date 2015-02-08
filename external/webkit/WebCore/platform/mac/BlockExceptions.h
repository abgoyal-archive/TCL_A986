

#import <Foundation/NSException.h>

void ReportBlockedObjCException(NSException *);

#define BEGIN_BLOCK_OBJC_EXCEPTIONS @try {
#define END_BLOCK_OBJC_EXCEPTIONS } @catch(NSException *localException) { ReportBlockedObjCException(localException); }

