

#ifndef WindowFeatures_h
#define WindowFeatures_h

#include "PlatformString.h"
#include <wtf/HashMap.h>

namespace WebCore {

    struct WindowFeatures {
        WindowFeatures()
            : xSet(false)
            , ySet(false)
            , widthSet(false)
            , heightSet(false)
            , menuBarVisible(true)
            , statusBarVisible(true)
            , toolBarVisible(true)
            , locationBarVisible(true)
            , scrollbarsVisible(true)
            , resizable(true)
            , fullscreen(false)
            , dialog(false)
        {
        }

        WindowFeatures(const String& features);

        void setWindowFeature(const String& keyString, const String& valueString);

        static bool boolFeature(const HashMap<String, String>& features, const char* key, bool defaultValue = false);
        static float floatFeature(const HashMap<String, String>& features, const char* key, float min, float max, float defaultValue);

        float x;
        bool xSet;
        float y;
        bool ySet;
        float width;
        bool widthSet;
        float height;
        bool heightSet;

        bool menuBarVisible;
        bool statusBarVisible;
        bool toolBarVisible;
        bool locationBarVisible;
        bool scrollbarsVisible;
        bool resizable;

        bool fullscreen;
        bool dialog;
    };

} // namespace WebCore

#endif // WindowFeatures_h
