

#ifndef HistoryController_h
#define HistoryController_h

#include "FrameLoaderTypes.h"
#include "PlatformString.h"
#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class HistoryItem;
class SerializedScriptValue;

class HistoryController : public Noncopyable {
public:
    HistoryController(Frame*);
    ~HistoryController();

    void saveScrollPositionAndViewStateToItem(HistoryItem*);
    void restoreScrollPositionAndViewState();

    void updateBackForwardListForFragmentScroll();

    void saveDocumentState();
    void saveDocumentAndScrollState();
    void restoreDocumentState();

    void invalidateCurrentItemCachedPage();

    void goToItem(HistoryItem*, FrameLoadType);
    bool urlsMatchItem(HistoryItem*) const;

    void updateForBackForwardNavigation();
    void updateForReload();
    void updateForStandardLoad();
    void updateForRedirectWithLockedBackForwardList();
    void updateForClientRedirect();
    void updateForCommit();
    void updateForSameDocumentNavigation();
    void updateForFrameLoadCompleted();

    HistoryItem* currentItem() const { return m_currentItem.get(); }
    void setCurrentItem(HistoryItem*);
    void setCurrentItemTitle(const String&);

    HistoryItem* provisionalItem() const { return m_provisionalItem.get(); }
    void setProvisionalItem(HistoryItem*);

    void pushState(PassRefPtr<SerializedScriptValue>, const String& title, const String& url);
    void replaceState(PassRefPtr<SerializedScriptValue>, const String& title, const String& url);

private:
    PassRefPtr<HistoryItem> createItem(bool useOriginal);
    PassRefPtr<HistoryItem> createItemTree(Frame* targetFrame, bool clipAtTarget);

    void recursiveGoToItem(HistoryItem*, HistoryItem*, FrameLoadType);
    bool childFramesMatchItem(HistoryItem*) const;
    void updateBackForwardListClippedAtTarget(bool doClip);

    Frame* m_frame;

    RefPtr<HistoryItem> m_currentItem;
    RefPtr<HistoryItem> m_previousItem;
    RefPtr<HistoryItem> m_provisionalItem;
};

} // namespace WebCore

#endif // HistoryController_h
