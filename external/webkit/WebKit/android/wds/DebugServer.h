

#ifndef DEBUGSERVER_H
#define DEBUGSERVER_H

// Turn on the wds feature in webkit
#define ENABLE_WDS 0

#include "wtf/Threading.h"
#include "wtf/Vector.h"

// Forward declarations.
namespace WebCore {
    class Frame;
}

using namespace WTF;
using namespace WebCore;

namespace android {

// WebCore Debug Server
namespace WDS {

class DebugServer : WTFNoncopyable::Noncopyable {
public:
    void start();
    void addFrame(Frame* frame) {
        m_frames.append(frame);
    }
    void removeFrame(Frame* frame) {
        size_t i = m_frames.find(frame);
        if (i != notFound)
            m_frames.remove(i);
    }
    Frame* getFrame(unsigned idx) {
        if (idx < m_frames.size())
            return m_frames.at(idx);
        return NULL;
    }
private:
    DebugServer();
    WTF::Vector<Frame*> m_frames;
    ThreadIdentifier m_threadId;
    friend DebugServer* server();
};

DebugServer* server();

} // end namespace WDS

} // end namespace android

#endif
