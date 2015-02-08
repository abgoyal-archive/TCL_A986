

#include "config.h"

#include "CachedFramePlatformDataAndroid.h"
#include "Settings.h"

namespace android {

CachedFramePlatformDataAndroid::CachedFramePlatformDataAndroid(WebCore::Settings* settings)
{
#ifdef ANDROID_META_SUPPORT
    m_viewport_width = settings->viewportWidth();
    m_viewport_height = settings->viewportHeight();
    m_viewport_initial_scale = settings->viewportInitialScale();
    m_viewport_minimum_scale = settings->viewportMinimumScale();
    m_viewport_maximum_scale = settings->viewportMaximumScale();
    m_viewport_target_densitydpi = settings->viewportTargetDensityDpi();
    m_viewport_user_scalable = settings->viewportUserScalable();
    m_format_detection_address = settings->formatDetectionAddress();
    m_format_detection_email = settings->formatDetectionEmail();
    m_format_detection_telephone = settings->formatDetectionTelephone();
#endif

}

#ifdef ANDROID_META_SUPPORT
void CachedFramePlatformDataAndroid::restoreMetadata(WebCore::Settings* settings)
{
    settings->setViewportWidth(m_viewport_width);
    settings->setViewportHeight(m_viewport_height);
    settings->setViewportInitialScale(m_viewport_initial_scale);
    settings->setViewportMinimumScale(m_viewport_minimum_scale);
    settings->setViewportMaximumScale(m_viewport_maximum_scale);
    settings->setViewportTargetDensityDpi(m_viewport_target_densitydpi);
    settings->setViewportUserScalable(m_viewport_user_scalable);
    settings->setFormatDetectionAddress(m_format_detection_address);
    settings->setFormatDetectionEmail(m_format_detection_email);
    settings->setFormatDetectionTelephone(m_format_detection_telephone);
}
#endif

}
