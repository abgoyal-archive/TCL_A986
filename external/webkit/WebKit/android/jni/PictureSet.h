

#ifndef PICTURESET_H
#define PICTURESET_H

#define PICTURE_SET_DUMP 0
#define PICTURE_SET_DEBUG 0
#define PICTURE_SET_VALIDATE 0

#if PICTURE_SET_DEBUG
#define DBG_SET_LOG(message) LOGD("%s %s", __FUNCTION__, message)
#define DBG_SET_LOGD(format, ...) LOGD("%s " format, __FUNCTION__, __VA_ARGS__)
#define DEBUG_SET_UI_LOGD(...) LOGD(__VA_ARGS__)
#else
#define DBG_SET_LOG(message) ((void)0)
#define DBG_SET_LOGD(format, ...) ((void)0)
#define DEBUG_SET_UI_LOGD(...) ((void)0)
#endif

#include "jni.h"
#include "SkRegion.h"
#include <wtf/Vector.h>

class SkCanvas;
class SkPicture;
class SkIRect;

namespace android {

    class PictureSet {
    public:
        PictureSet();
        PictureSet(const PictureSet& src) { set(src); }
        virtual ~PictureSet();
        void add(const SkRegion& area, SkPicture* picture,
            uint32_t elapsed, bool split) 
        {
            add(area, picture, elapsed, split, emptyPicture(picture));
        }
        void add(const SkRegion& area, SkPicture* picture,
            uint32_t elapsed, bool split, bool empty);
        const SkIRect& bounds(size_t i) const {
            return mPictures[i].mArea.getBounds(); }
        bool build();
        // Update mWidth/mHeight, and adds any additional inval region
        void checkDimensions(int width, int height, SkRegion* inval);
        void clear();
        bool draw(SkCanvas* );
        static PictureSet* GetNativePictureSet(JNIEnv* env, jobject jpic);
        int height() const { return mHeight; }
        bool isEmpty() const; // returns true if empty or only trivial content
        bool reuseSubdivided(const SkRegion& );
        void set(const PictureSet& );
        void setDrawTimes(const PictureSet& );
        void setPicture(size_t i, SkPicture* p);
        size_t size() const { return mPictures.size(); }
        void split(PictureSet* result) const;
        bool upToDate(size_t i) const { return mPictures[i].mPicture != NULL; }
        int width() const { return mWidth; }
        void dump(const char* label) const;
        bool validate(const char* label) const;
    private:
        bool emptyPicture(SkPicture* ) const; // true if no text, images, paths
        struct Pictures {
            SkRegion mArea;
            SkPicture* mPicture;
            SkIRect mUnsplit;
            uint32_t mElapsed;
            bool mSplit : 8;
            bool mWroteElapsed : 8;
            bool mBase : 8; // true if nothing is drawn underneath this
            bool mEmpty : 8; // true if the picture only draws white
        };
        void add(const Pictures* temp);
        WTF::Vector<Pictures> mPictures;
        int mHeight;
        int mWidth;
    };
}

#endif
