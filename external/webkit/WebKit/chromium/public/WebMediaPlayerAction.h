

#ifndef WebMediaPlayerAction_h
#define WebMediaPlayerAction_h

namespace WebKit {

struct WebMediaPlayerAction {
    enum Type {
        Unknown,
        Play,
        Mute,
        Loop
    };

    Type type;
    bool enable;

    WebMediaPlayerAction()
        : type(Unknown), enable(false) { }
    WebMediaPlayerAction(Type type, bool enable)
        : type(type), enable(enable) { }
};

} // namespace WebKit

#endif
