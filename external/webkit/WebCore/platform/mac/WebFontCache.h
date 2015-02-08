

#import <AppKit/NSFontManager.h>
#import <wtf/Vector.h>

// This interface exists so that third party products (like Silk) can patch in to an Obj-C method to manipulate WebKit's font caching/substitution.
@interface WebFontCache : NSObject
+ (NSFont *)fontWithFamily:(NSString *)desiredFamily traits:(NSFontTraitMask)desiredTraits weight:(int)desiredWeight size:(float)size;
+ (void)getTraits:(Vector<unsigned>&)traitsMasks inFamily:(NSString *)desiredFamily;

// This older version of the interface is relied upon by some clients. WebCore doesn't use it.
+ (NSFont *)fontWithFamily:(NSString *)desiredFamily traits:(NSFontTraitMask)desiredTraits size:(float)size;
@end
