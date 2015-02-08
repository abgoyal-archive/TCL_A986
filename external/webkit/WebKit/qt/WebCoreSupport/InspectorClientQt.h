

#ifndef InspectorClientQt_h
#define InspectorClientQt_h

#include "InspectorClient.h"
#include "OwnPtr.h"
#include <QtCore/QString>

class QWebPage;
class QWebView;

namespace WebCore {
    class Node;
    class Page;
    class String;

    class InspectorClientQt : public InspectorClient {
    public:
        InspectorClientQt(QWebPage*);

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

    private:
        void updateWindowTitle();
        QWebPage* m_inspectedWebPage;
        OwnPtr<QWebView> m_inspectorView;
        QString m_inspectedURL;
    };
}

#endif
