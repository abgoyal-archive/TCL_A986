

#ifndef ProgressTracker_h
#define ProgressTracker_h

#include <wtf/HashMap.h>
#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class ResourceResponse;
struct ProgressItem;

class ProgressTracker : public Noncopyable {
public:
    ProgressTracker();
    ~ProgressTracker();
    
    unsigned long createUniqueIdentifier();

    double estimatedProgress() const;

    void progressStarted(Frame*);
    void progressCompleted(Frame*);
    
    void incrementProgress(unsigned long identifier, const ResourceResponse&);
    void incrementProgress(unsigned long identifier, const char*, int);
    void completeProgress(unsigned long identifier);

    long long totalPageAndResourceBytesToLoad() const { return m_totalPageAndResourceBytesToLoad; }
    long long totalBytesReceived() const { return m_totalBytesReceived; }

private:
    void reset();
    void finalProgressComplete();
    
    unsigned long m_uniqueIdentifier;
    
    long long m_totalPageAndResourceBytesToLoad;
    long long m_totalBytesReceived;
    double m_lastNotifiedProgressValue;
    double m_lastNotifiedProgressTime;
    double m_progressNotificationInterval;
    double m_progressNotificationTimeInterval;
    bool m_finalProgressChangedSent;    
    double m_progressValue;
    RefPtr<Frame> m_originatingProgressFrame;
    
    int m_numProgressTrackedFrames;
    HashMap<unsigned long, ProgressItem*> m_progressItems;
};
    
}

#endif
