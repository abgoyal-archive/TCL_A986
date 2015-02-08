

#ifdef __OBJC__

#import <Foundation/Foundation.h>

#endif

// If we don't define these, they get defined in windef.h. 
// We want to use std::min and std::max
#define max max
#define min min
