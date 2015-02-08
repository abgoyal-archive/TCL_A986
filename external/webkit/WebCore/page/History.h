

#ifndef History_h
#define History_h

#include "KURL.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class Frame;
class SerializedScriptValue;
class String;
typedef int ExceptionCode;

class History : public RefCounted<History> {
public:
    static PassRefPtr<History> create(Frame* frame) { return adoptRef(new History(frame)); }
    
    Frame* frame() const;
    void disconnectFrame();

    unsigned length() const;
    void back();
    void forward();
    void go(int distance);

    enum StateObjectType {
        StateObjectPush,
        StateObjectReplace
    };
    void stateObjectAdded(PassRefPtr<SerializedScriptValue>, const String& title, const String& url, StateObjectType, ExceptionCode&);

private:
    History(Frame*);

    KURL urlForState(const String& url);

    Frame* m_frame;
};

} // namespace WebCore

#endif // History_h
