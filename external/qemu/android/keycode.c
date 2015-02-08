
#include "android/keycode.h"

AndroidKeyCode
android_keycode_rotate( AndroidKeyCode  code, int  rotation )
{
    static const AndroidKeyCode  wheel[4] = { kKeyCodeDpadUp,
                                              kKeyCodeDpadRight,
                                              kKeyCodeDpadDown,
                                              kKeyCodeDpadLeft };

    int  index;

    for (index = 0; index < 4; index++) {
        if (code == wheel[index]) {
            index = (index + rotation) & 3;
            code  = wheel[index];
            break;
        }
    }
    return code;
}

