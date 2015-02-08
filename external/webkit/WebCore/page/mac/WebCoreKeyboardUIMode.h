

#ifndef WebCoreKeyboardUIMode_h
#define WebCoreKeyboardUIMode_h

namespace WebCore {

    enum KeyboardUIMode {
        KeyboardAccessDefault     = 0x00000000,
        KeyboardAccessFull        = 0x00000001,
        // this flag may be or'ed with either of the two above
        KeyboardAccessTabsToLinks = 0x10000000
    };

}

#endif
