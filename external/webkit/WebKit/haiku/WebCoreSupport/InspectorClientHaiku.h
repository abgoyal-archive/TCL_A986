

#ifndef InspectorClientHaiku_h
#define InspectorClientHaiku_h

#include "InspectorClient.h"


namespace WebCore {
    class Node;
    class Page;
    class String;

    class InspectorClientHaiku : public InspectorClient {
    public:
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

        virtual void populateSetting(const WebCore::String& key, WebCore::String* value);
        virtual void storeSetting(const WebCore::String& key, const WebCore::String& value);

        virtual void inspectorWindowObjectCleared();
    };
} // namespace WebCore

#endif // InspectorClientHaiku_h

