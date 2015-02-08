


#ifndef MediaPlayerProxy_h
#define MediaPlayerProxy_h

#if ENABLE(VIDEO)

#include "ScriptInstance.h"

namespace WebCore {

    class IntRect;
    class IntSize;
    class String;
    class MediaPlayer;
    class PluginView;
    class HTMLMediaElement;

    enum MediaPlayerProxyNotificationType {
        MediaPlayerNotificationPlayPauseButtonPressed,
        Idle,
        Loading,
        Loaded,
        FormatError,
        NetworkError,
        DecodeError
    };

    class WebMediaPlayerProxy {
    public:
        WebMediaPlayerProxy(MediaPlayer* player);
        ~WebMediaPlayerProxy();

        MediaPlayer* mediaPlayer() {return m_mediaPlayer;}
        void initEngine();
        void load(const String& url);
        HTMLMediaElement* element();
        void invokeMethod(const String& methodName);
        ScriptInstance pluginInstance();

    private:
        MediaPlayer* m_mediaPlayer;
        bool m_init;
        WebCore::PluginView* m_pluginView;
        bool m_hasSentResponseToPlugin;
        ScriptInstance m_instance;
    };

}
#endif // ENABLE(VIDEO)

#endif
