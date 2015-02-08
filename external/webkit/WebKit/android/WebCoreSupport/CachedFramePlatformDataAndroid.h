

#ifndef CachedFramePlatformDatatAndroid_h
#define CachedFramePlatformDatatAndroid_h

#include "CachedFramePlatformData.h"

namespace WebCore {
    class Settings;
}

namespace android {

class CachedFramePlatformDataAndroid : public WebCore::CachedFramePlatformData {
public:
    CachedFramePlatformDataAndroid(WebCore::Settings* settings);

#ifdef ANDROID_META_SUPPORT
    void restoreMetadata(WebCore::Settings* settings);
#endif

private:
#ifdef ANDROID_META_SUPPORT
    // meta data of the frame
    int m_viewport_width;
    int m_viewport_height;
    int m_viewport_initial_scale;
    int m_viewport_minimum_scale;
    int m_viewport_maximum_scale;
    int m_viewport_target_densitydpi;
    bool m_viewport_user_scalable : 1;
    bool m_format_detection_address : 1;
    bool m_format_detection_email : 1;
    bool m_format_detection_telephone : 1;
#endif
};

}

#endif