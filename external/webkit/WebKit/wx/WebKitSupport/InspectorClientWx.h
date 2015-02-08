

#ifndef InspectorClientWx_h
#define InspectorClientWx_h

#include "InspectorClient.h"

namespace WebCore {

class Node;
class Page;
class String;

class InspectorClientWx : public InspectorClient {
public:
    InspectorClientWx();
    ~InspectorClientWx();

    virtual void inspectorDestroyed();

    virtual Page* createPage();

    virtual String localizedStringsURL();

    virtual String hiddenPanels();

    virtual void showWindow();
    virtual void closeWindow();

    virtual void attachWindow();
    virtual void detachWindow();

    virtual void setAttachedWindowHeight(unsigned height);

    virtual void highlight(Node*);
    virtual void hideHighlight();

    virtual void inspectedURLChanged(const String& newURL);

    virtual void populateSetting(const String& key, String* value);
    virtual void storeSetting(const String& key, const String& value);

    virtual void inspectorWindowObjectCleared();
};

} // namespace WebCore

#endif // !defined(InspectorClient_h)
