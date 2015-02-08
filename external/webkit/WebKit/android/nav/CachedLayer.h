

#ifndef CachedLayer_H
#define CachedLayer_H

#include "CachedDebug.h"
#include "IntRect.h"

class SkPicture;

namespace WebCore {
    class LayerAndroid;
}

using namespace WebCore;

namespace android {

class CachedLayer {
public:
#if USE(ACCELERATED_COMPOSITING)
    bool operator<(const CachedLayer& l) const {
        return mCachedNodeIndex < l.mCachedNodeIndex;
    }
    // FIXME: adjustBounds should be renamed globalBounds or toGlobal
    IntRect adjustBounds(const LayerAndroid* root, const IntRect& bounds) const;
    int cachedNodeIndex() const { return mCachedNodeIndex; }
    const IntPoint& getOffset() const { return mOffset; }
    const LayerAndroid* layer(const LayerAndroid* root) const;
    IntRect localBounds(const IntRect& bounds) const;
    SkPicture* picture(const LayerAndroid* root) const;
    void reset() { mLayer = 0; }
    void setCachedNodeIndex(int index) { mCachedNodeIndex = index; }
    void setOffset(const IntPoint& offset) { mOffset = offset; }
    void setUniqueId(int uniqueId) { mUniqueId = uniqueId; }
    int uniqueId() const { return mUniqueId; }
private:
    int mCachedNodeIndex;
    mutable const LayerAndroid* mLayer;
    IntPoint mOffset;
    int mUniqueId;

#if DUMP_NAV_CACHE
public:
    class Debug {
public:
        CachedLayer* base() const;
        void print() const;
        static void printLayerAndroid(const LayerAndroid* );
        static void printRootLayerAndroid(const LayerAndroid* );
        static int spaces;
    } mDebug;
#endif
#endif // USE(ACCELERATED_COMPOSITING)
};

}

#endif
