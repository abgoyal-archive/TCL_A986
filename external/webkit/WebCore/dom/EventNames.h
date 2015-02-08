

#ifndef EventNames_h
#define EventNames_h

#include "AtomicString.h"
#include "ThreadGlobalData.h"

namespace WebCore {

#define DOM_EVENT_NAMES_FOR_EACH(macro) \
    \
    macro(abort) \
    macro(beforecopy) \
    macro(beforecut) \
    macro(beforeload) \
    macro(beforepaste) \
    macro(beforeunload) \
    macro(blur) \
    macro(cached) \
    macro(change) \
    macro(checking) \
    macro(click) \
    macro(close) \
    macro(compositionend) \
    macro(compositionstart) \
    macro(compositionupdate) \
    macro(connect) \
    macro(contextmenu) \
    macro(copy) \
    macro(cut) \
    macro(dblclick) \
    macro(display) \
    macro(downloading) \
    macro(drag) \
    macro(dragend) \
    macro(dragenter) \
    macro(dragleave) \
    macro(dragover) \
    macro(dragstart) \
    macro(drop) \
    macro(error) \
    macro(focus) \
    macro(hashchange) \
    macro(input) \
    macro(invalid) \
    macro(keydown) \
    macro(keypress) \
    macro(keyup) \
    macro(load) \
    macro(loadstart) \
    macro(message) \
    macro(mousedown) \
    macro(mousemove) \
    macro(mouseout) \
    macro(mouseover) \
    macro(mouseup) \
    macro(mousewheel) \
    macro(noupdate) \
    macro(obsolete) \
    macro(offline) \
    macro(online) \
    macro(open) \
    macro(overflowchanged) \
    macro(pagehide) \
    macro(pageshow) \
    macro(paste) \
    macro(popstate) \
    macro(readystatechange) \
    macro(reset) \
    macro(resize) \
    macro(scroll) \
    macro(search) \
    macro(select) \
    macro(selectstart) \
    macro(storage) \
    macro(submit) \
    macro(textInput) \
    macro(unload) \
    macro(updateready) \
    macro(zoom) \
    \
    macro(DOMActivate) \
    macro(DOMAttrModified) \
    macro(DOMCharacterDataModified) \
    macro(DOMFocusIn) \
    macro(DOMFocusOut) \
    macro(DOMNodeInserted) \
    macro(DOMNodeInsertedIntoDocument) \
    macro(DOMNodeRemoved) \
    macro(DOMNodeRemovedFromDocument) \
    macro(DOMSubtreeModified) \
    macro(DOMContentLoaded) \
    \
    macro(webkitBeforeTextInserted) \
    macro(webkitEditableContentChanged) \
    \
    macro(canplay) \
    macro(canplaythrough) \
    macro(durationchange) \
    macro(emptied) \
    macro(ended) \
    macro(loadeddata) \
    macro(loadedmetadata) \
    macro(pause) \
    macro(play) \
    macro(playing) \
    macro(ratechange) \
    macro(seeked) \
    macro(seeking) \
    macro(timeupdate) \
    macro(volumechange) \
    macro(waiting) \
    \
    macro(webkitbeginfullscreen) \
    macro(webkitendfullscreen) \
    \
    macro(progress) \
    macro(stalled) \
    macro(suspend) \
    \
    macro(webkitAnimationEnd) \
    macro(webkitAnimationStart) \
    macro(webkitAnimationIteration) \
    \
    macro(webkitTransitionEnd) \
    \
    macro(orientationchange) \
    \
    macro(touchstart) \
    macro(touchmove) \
    macro(touchend) \
    macro(touchcancel) \
/* #if PLATFORM(ANDROID) */ \
    macro(touchlongpress) \
    macro(touchdoubletap) \
/* #endif */ \
    \
    macro(success) \
    \
// end of DOM_EVENT_NAMES_FOR_EACH

    class EventNames : public Noncopyable {
        int dummy; // Needed to make initialization macro work.
        // Private to prevent accidental call to EventNames() instead of eventNames()
        EventNames();
        friend class ThreadGlobalData;

    public:
        #define DOM_EVENT_NAMES_DECLARE(name) AtomicString name##Event;
        DOM_EVENT_NAMES_FOR_EACH(DOM_EVENT_NAMES_DECLARE)
        #undef DOM_EVENT_NAMES_DECLARE
    };

    inline EventNames& eventNames()
    {
        return threadGlobalData().eventNames();
    }

}

#endif
