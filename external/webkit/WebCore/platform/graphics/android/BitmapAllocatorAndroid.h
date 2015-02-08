

#ifndef WebCore_BitmapAllocatorAndroid_DEFINED
#define WebCore_BitmapAllocatorAndroid_DEFINED

#include "SkBitmap.h"

namespace WebCore {

    class SharedBuffer;
    class SharedBufferStream;

    /** Returns a custom allocator that takes advantage of ashmem and global
        pools to best manage the pixel memory for a decoded image. This should
        be used for images that are logically immutable, and can be re-decoded
        at will based on available memory.
     */
    class BitmapAllocatorAndroid : public SkBitmap::Allocator {
    public:
        BitmapAllocatorAndroid(SharedBuffer* data, int sampleSize);
        virtual ~BitmapAllocatorAndroid();

        // overrides
        virtual bool allocPixelRef(SkBitmap*, SkColorTable*);

    private:
        SharedBufferStream* fStream;
        int                 fSampleSize;
    };

}

#endif
