

#include "config.h"
#include "LayoutTestController.h"

#include "DumpRenderTree.h"
#include "WorkQueue.h"
#include "WorkQueueItem.h"
#include <JavaScriptCore/JSRetainPtr.h>
#include <JavaScriptCore/JSStringRef.h>



LayoutTestController::~LayoutTestController()
{
    // FIXME: implement
}

void LayoutTestController::addDisallowedURL(JSStringRef url)
{
    // FIXME: implement
}

void LayoutTestController::clearBackForwardList()
{
}

JSStringRef LayoutTestController::copyDecodedHostName(JSStringRef name)
{
    // FIXME: implement
    return 0;
}

JSStringRef LayoutTestController::copyEncodedHostName(JSStringRef name)
{
    // FIXME: implement
    return 0;
}

void LayoutTestController::dispatchPendingLoadRequests()
{
    // FIXME: Implement for testing fix for 6727495
}

void LayoutTestController::display()
{
}

void LayoutTestController::keepWebHistory()
{
    // FIXME: implement
}

void LayoutTestController::notifyDone()
{
    if (m_waitToDump && !WorkQueue::shared()->count())
        notifyDoneFired();
    m_waitToDump = false;
}

JSStringRef LayoutTestController::pathToLocalResource(JSContextRef context, JSStringRef url)
{
    // Function introduced in r28690. This may need special-casing on Windows.
    return JSStringRetain(url); // Do nothing on Unix.
}

void LayoutTestController::queueLoad(JSStringRef url, JSStringRef target)
{
    // FIXME: We need to resolve relative URLs here
    WorkQueue::shared()->queue(new LoadItem(url, target));
}

void LayoutTestController::setAcceptsEditing(bool acceptsEditing)
{
}

void LayoutTestController::setAlwaysAcceptCookies(bool alwaysAcceptCookies)
{
    // FIXME: Implement this (and restore the default value before running each test in DumpRenderTree.cpp).
}

void LayoutTestController::setCustomPolicyDelegate(bool, bool)
{
    // FIXME: implement
}

void LayoutTestController::setMainFrameIsFirstResponder(bool flag)
{
    // FIXME: implement
}

void LayoutTestController::setTabKeyCyclesThroughElements(bool cycles)
{
    // FIXME: implement
}

void LayoutTestController::setUseDashboardCompatibilityMode(bool flag)
{
    // FIXME: implement
}

void LayoutTestController::setUserStyleSheetEnabled(bool flag)
{
}

void LayoutTestController::setUserStyleSheetLocation(JSStringRef path)
{
}

void LayoutTestController::setWindowIsKey(bool windowIsKey)
{
    // FIXME: implement
}

void LayoutTestController::setSmartInsertDeleteEnabled(bool flag)
{
    // FIXME: implement
}

void LayoutTestController::setJavaScriptProfilingEnabled(bool flag)
{
}

void LayoutTestController::setWaitToDump(bool waitUntilDone)
{
    static const int timeoutSeconds = 10;

    m_waitToDump = waitUntilDone;
}

int LayoutTestController::windowCount()
{
    // FIXME: implement
    return 1;
}

void LayoutTestController::setPrivateBrowsingEnabled(bool privateBrowsingEnabled)
{
    // FIXME: implement
}

void LayoutTestController::setXSSAuditorEnabled(bool enabled)
{
    // FIXME: implement
}

void LayoutTestController::setFrameSetFlatteningEnabled(bool enabled)
{
    // FIXME: implement
}

void LayoutTestController::setAllowUniversalAccessFromFileURLs(bool enabled)
{
    // FIXME: implement
}

void LayoutTestController::setAuthorAndUserStylesEnabled(bool flag)
{
    // FIXME: implement
}

void LayoutTestController::setPopupBlockingEnabled(bool popupBlockingEnabled)
{
    // FIXME: implement
}

bool LayoutTestController::elementDoesAutoCompleteForElementWithId(JSStringRef id) 
{
    // FIXME: implement
    return false;
}

void LayoutTestController::execCommand(JSStringRef name, JSStringRef value)
{
    // FIXME: implement
}

void LayoutTestController::setPersistentUserStyleSheetLocation(JSStringRef jsURL)
{
    // FIXME: implement
}

void LayoutTestController::clearPersistentUserStyleSheet()
{
    // FIXME: implement
}

void LayoutTestController::clearAllDatabases()
{
    // FIXME: implement
}
 
void LayoutTestController::setDatabaseQuota(unsigned long long quota)
{    
    // FIXME: implement
}

void LayoutTestController::setDomainRelaxationForbiddenForURLScheme(bool, JSStringRef)
{
    // FIXME: implement
}

void LayoutTestController::setAppCacheMaximumSize(unsigned long long size)
{
    // FIXME: implement
}

unsigned LayoutTestController::numberOfActiveAnimations() const
{
    // FIXME: implement
    return 0;
}

unsigned LayoutTestController::workerThreadCount() const
{
    // FIXME: implement
    return 0;
}

void LayoutTestController::setSelectTrailingWhitespaceEnabled(bool flag)
{
    // FIXME: implement
}

bool LayoutTestController::pauseTransitionAtTimeOnElementWithId(JSStringRef propertyName, double time, JSStringRef elementId)
{
    // FIXME: implement
    return false;
}

void LayoutTestController::setMockGeolocationPosition(double latitude, double longitude, double accuracy)
{
    // FIXME: Implement for Geolocation layout tests.
    // See https://bugs.webkit.org/show_bug.cgi?id=28264.
}

void LayoutTestController::setMockGeolocationError(int code, JSStringRef message)
{
    // FIXME: Implement for Geolocation layout tests.
    // See https://bugs.webkit.org/show_bug.cgi?id=28264.
}

void LayoutTestController::setIconDatabaseEnabled(bool iconDatabaseEnabled)
{
    // FIXME: implement
}

bool LayoutTestController::pauseAnimationAtTimeOnElementWithId(JSStringRef animationName, double time, JSStringRef elementId)
{
    // FIXME: implement
    return false;
}

bool LayoutTestController::sampleSVGAnimationForElementAtTime(JSStringRef animationId, double time, JSStringRef elementId)
{
    // FIXME: implement
    return false;
}

void LayoutTestController::setCacheModel(int)
{
    // FIXME: implement
}

bool LayoutTestController::isCommandEnabled(JSStringRef /*name*/)
{
    // FIXME: implement
    return false;
}

size_t LayoutTestController::webHistoryItemCount()
{
    // FIXME: implement
    return 0;
}

void LayoutTestController::waitForPolicyDelegate()
{
    // FIXME: Implement this.
}

void LayoutTestController::overridePreference(JSStringRef /* key */, JSStringRef /* value */)
{
    // FIXME: implement
}

void LayoutTestController::addUserScript(JSStringRef source, bool runAtStart)
{
    printf("LayoutTestController::addUserScript not implemented.\n");
}

void LayoutTestController::addUserStyleSheet(JSStringRef source)
{
    printf("LayoutTestController::addUserStyleSheet not implemented.\n");
}

void LayoutTestController::showWebInspector()
{
    // FIXME: Implement this.
}

void LayoutTestController::closeWebInspector()
{
    // FIXME: Implement this.
}

void LayoutTestController::evaluateInWebInspector(long callId, JSStringRef script)
{
    // FIXME: Implement this.
}

void LayoutTestController::removeAllVisitedLinks()
{
    // FIXME: Implement this.
}

void LayoutTestController::setTimelineProfilingEnabled(bool enabled)
{

}

void LayoutTestController::evaluateScriptInIsolatedWorld(unsigned worldId, JSObjectRef globalObject, JSStringRef script)
{

}

void LayoutTestController::disableImageLoading()
{

}

void LayoutTestController::whiteListAccessFromOrigin(JSStringRef sourceOrigin, JSStringRef destinationProtocol, JSStringRef destinationHost, bool allowDestinationSubdomains)
{

}

JSRetainPtr<JSStringRef> LayoutTestController::counterValueForElementById(JSStringRef id)
{
    return 0;
}

int LayoutTestController::pageNumberForElementById(JSStringRef, float, float)
{
    // FIXME: implement
    return -1;
}

int LayoutTestController::numberOfPages(float, float)
{
    // FIXME: implement
    return -1;
}