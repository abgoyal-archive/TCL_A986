

#ifndef ANDROID_WEBKIT_WEBHISTORY_H
#define ANDROID_WEBKIT_WEBHISTORY_H

#include "AndroidWebHistoryBridge.h"

#include <jni.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace android {

class AutoJObject;

class WebHistory {
public:
    static jbyteArray Flatten(JNIEnv*, WTF::Vector<char>&, WebCore::HistoryItem*);
    static void AddItem(const AutoJObject&, WebCore::HistoryItem*);
    static void RemoveItem(const AutoJObject&, int);
    static void UpdateHistoryIndex(const AutoJObject&, int);
};

// there are two scale factors saved with each history item. mScale reflects the
// viewport scale factor, default to 100 means 100%. mScreenWidthScale records
// the scale factor for the screen width used to wrap the text paragraph.
class WebHistoryItem : public WebCore::AndroidWebHistoryBridge {
public:
    WebHistoryItem(WebHistoryItem* parent)
        : WebCore::AndroidWebHistoryBridge(0)
        , m_parent(parent)
        , m_object(NULL) { }
    WebHistoryItem(JNIEnv*, jobject, WebCore::HistoryItem*);
    ~WebHistoryItem();
    void updateHistoryItem(WebCore::HistoryItem* item);
    void setParent(WebHistoryItem* parent) { m_parent = parent; }
    WebHistoryItem* parent() const { return m_parent.get(); }
private:
    RefPtr<WebHistoryItem> m_parent;
    jweak m_object;
};

};

#endif
