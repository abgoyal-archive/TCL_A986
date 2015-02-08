

#ifndef HistoryPropertyList_h
#define HistoryPropertyList_h

#include "BinaryPropertyList.h"
#include "PlatformString.h"
#include <wtf/RetainPtr.h>

namespace WebCore {

class HistoryItem;

class HistoryPropertyListWriter : public BinaryPropertyListWriter {
public:
    RetainPtr<CFDataRef> releaseData();

protected:
    HistoryPropertyListWriter();

    void writeHistoryItem(BinaryPropertyListObjectStream&, HistoryItem*);

private:
    virtual void writeHistoryItems(BinaryPropertyListObjectStream&) = 0;

    virtual void writeObjects(BinaryPropertyListObjectStream&);
    virtual UInt8* buffer(size_t);

    const String m_dailyVisitCountsKey;
    const String m_displayTitleKey;
    const String m_lastVisitWasFailureKey;
    const String m_lastVisitWasHTTPNonGetKey;
    const String m_lastVisitedDateKey;
    const String m_redirectURLsKey;
    const String m_titleKey;
    const String m_urlKey;
    const String m_visitCountKey;
    const String m_weeklyVisitCountsKey;

    UInt8* m_buffer;
    size_t m_bufferSize;
};

}

#endif
