

#include "PluginDebugAndroid.h"
#include "utils/Log.h"
#include "utils/SystemClock.h"
#include <stdarg.h>

#define ARRAY_COUNT(array) static_cast<int32_t>(sizeof(array) / sizeof(array[0]))

// used for key, mouse, and touch inputs
static const char* const inputActions[] = {
    "down",
    "up",
    "move",         /* touch only */
    "cancel",       /* touch only */
    "longPress",    /* touch only */
    "doubleTap"     /* touch only */
};

static const char* const lifecycleActions[] = {
    "kPause_ANPLifecycleAction",
    "kResume_ANPLifecycleAction",
    "kGainFocus_ANPLifecycleAction",
    "kLoseFocus_ANPLifecycleAction",
    "kFreeMemory_ANPLifecycleAction",
    "kOnLoad_ANPLifecycleAction",
    "kEnterFullScreen_ANPLifecycleAction",
    "kExitFullScreen_ANPLifecycleAction",
    "kOnScreen_ANPLifecycleAction",
    "kOffScreen_ANPLifecycleAction"
};

void anp_logPlugin(const char format[], ...) {
    va_list args;
    va_start(args, format);
    LOG_PRI_VA(ANDROID_LOG_DEBUG, "webkit_plugin", format, args);
    va_end(args);
}

void anp_logPluginEvent(void* npp, const ANPEvent* evt, int16 returnVal, int elapsedTime) {

    switch(evt->eventType) {

        case kNull_ANPEventType:
            PLUGIN_LOG("%p EVENT::NULL", npp);
            break;

        case kKey_ANPEventType:
            if(evt->data.key.action < ARRAY_COUNT(inputActions)) {
                anp_logPlugin("%p EVENT::KEY[%d] time=%d action=%s code=%d vcode=%d unichar=%d repeat=%d mods=%x",
                        npp, returnVal, elapsedTime, inputActions[evt->data.key.action],
                        evt->data.key.nativeCode, evt->data.key.virtualCode,
                        evt->data.key.unichar, evt->data.key.repeatCount,
                        evt->data.key.modifiers);
            } else {
                PLUGIN_LOG("%p EVENT::KEY[%d] unknown action", npp, returnVal);
            }
            break;

        case kMouse_ANPEventType:
            if(evt->data.mouse.action < ARRAY_COUNT(inputActions)) {
                anp_logPlugin("%p EVENT::MOUSE[%d] time=%d action=%s [%d %d]", npp,
                        returnVal, elapsedTime, inputActions[evt->data.mouse.action],
                        evt->data.touch.x, evt->data.touch.y);
            } else {
                anp_logPlugin("%p EVENT::MOUSE[%d] unknown action", npp, returnVal);
            }
            break;

        case kTouch_ANPEventType:
            if(evt->data.touch.action < ARRAY_COUNT(inputActions)) {

                anp_logPlugin("%p EVENT::TOUCH[%d] time=%d action=%s [%d %d]",
                        npp, returnVal, elapsedTime,
                        inputActions[evt->data.touch.action], evt->data.touch.x,
                        evt->data.touch.y);
            } else {
                anp_logPlugin("%p EVENT::TOUCH[%d] unknown action", npp, returnVal);
            }
            break;

        case kDraw_ANPEventType:
            if (evt->data.draw.model == kBitmap_ANPDrawingModel) {
                anp_logPlugin("%p EVENT::DRAW bitmap time=%d format=%d clip=[%d,%d,%d,%d]",
                        npp, elapsedTime, evt->data.draw.data.bitmap.format,
                        evt->data.draw.clip.left, evt->data.draw.clip.top,
                        evt->data.draw.clip.right, evt->data.draw.clip.bottom);
            } else {
                anp_logPlugin("%p EVENT::DRAW unknown drawing model", npp);
            }
            break;

        case kLifecycle_ANPEventType:
            if(evt->data.lifecycle.action < ARRAY_COUNT(lifecycleActions)) {
                anp_logPlugin("%p EVENT::LIFECYCLE time=%d action=%s", npp, elapsedTime,
                        lifecycleActions[evt->data.lifecycle.action]);
            } else {
                anp_logPlugin("%p EVENT::LIFECYCLE unknown action", npp);
            }
            break;

        case kCustom_ANPEventType:
            anp_logPlugin("%p EVENT::CUSTOM time=%d", npp, elapsedTime);
            break;

        default:
            anp_logPlugin("%p EVENT::UNKNOWN", npp);
            break;
    }
}
