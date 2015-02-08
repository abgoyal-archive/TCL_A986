
#ifndef PluginContainerQt_H
#define PluginContainerQt_H

#include <QX11EmbedContainer>

namespace WebCore {

    class PluginView;

    class PluginContainerQt : public QX11EmbedContainer
    {
        Q_OBJECT
    public:
        PluginContainerQt(PluginView*, QWidget* parent);
        ~PluginContainerQt();

        void redirectWheelEventsToParent(bool enable = true);

    protected:
        virtual bool x11Event(XEvent*);
        virtual void focusInEvent(QFocusEvent*);
        virtual void focusOutEvent(QFocusEvent*);

    public slots:
        void on_clientClosed();
        void on_clientIsEmbedded();

    private:
        PluginView* m_pluginView;
        QWidget* m_clientWrapper;
    };

    class PluginClientWrapper : public QWidget
    {
    public:
        PluginClientWrapper(QWidget* parent, WId client);
        ~PluginClientWrapper();
        bool x11Event(XEvent*);

    private:
        QWidget* m_parent;
    };
}

#endif // PluginContainerQt_H
