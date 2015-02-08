

#ifndef InspectorClient_h
#define InspectorClient_h

#include "InspectorController.h"

namespace WebCore {

class Node;
class Page;
class String;

class InspectorClient {
public:
    virtual ~InspectorClient() {  }

    virtual void inspectorDestroyed() = 0;

    virtual Page* createPage() = 0;

    virtual String localizedStringsURL() = 0;

    virtual String hiddenPanels() = 0;

    virtual void showWindow() = 0;
    virtual void closeWindow() = 0;

    virtual void attachWindow() = 0;
    virtual void detachWindow() = 0;

    virtual void setAttachedWindowHeight(unsigned height) = 0;

    virtual void highlight(Node*) = 0;
    virtual void hideHighlight() = 0;

    virtual void inspectedURLChanged(const String& newURL) = 0;

    virtual void populateSetting(const String& key, String* value) = 0;
    virtual void storeSetting(const String& key, const String& value) = 0;

    virtual void inspectorWindowObjectCleared() = 0;
};

} // namespace WebCore

#endif // !defined(InspectorClient_h)
