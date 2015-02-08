

#ifndef PluginContainerSymbian_h
#define PluginContainerSymbian_h

#include <QWidget>

namespace WebCore {

    class PluginView;

    class PluginContainerSymbian : public QWidget {
        Q_OBJECT
    public:
        PluginContainerSymbian(PluginView*, QWidget* parent);
        ~PluginContainerSymbian();

        void requestGeometry(const QRect&, const QRegion& clip = QRegion());
        void adjustGeometry();

    protected:
        virtual void focusInEvent(QFocusEvent*);
        virtual void focusOutEvent(QFocusEvent*);
    private:
        PluginView* m_pluginView;
        QWidget* m_parent;
        QRect m_windowRect;
        QRegion m_clipRegion;
        bool m_hasPendingGeometryChange;
    };
}

#endif // PluginContainerSymbian_h
