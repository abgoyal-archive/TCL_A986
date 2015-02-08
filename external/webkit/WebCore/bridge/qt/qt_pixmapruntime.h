

#ifndef qt_pixmapruntime_h
#define qt_pixmapruntime_h

#include "Bridge.h"
#include <QVariant>

namespace JSC {

namespace Bindings {

class QtPixmapInstance : public Instance {
    QVariant data;
public:
    QtPixmapInstance(PassRefPtr<RootObject> rootObj, const QVariant& newData);
    virtual Class* getClass() const;
    virtual JSValue invokeMethod(ExecState*, const MethodList&, const ArgList& args);
    virtual void getPropertyNames(ExecState*, PropertyNameArray&);

    virtual JSValue defaultValue(ExecState*, PreferredPrimitiveType) const;
    virtual JSValue valueOf(ExecState* exec) const;
    int width() const;
    int height() const;
    QPixmap toPixmap();
    QImage toImage();
    static JSObject* createRuntimeObject(ExecState*, PassRefPtr<RootObject>, const QVariant&);
    static QVariant variantFromObject(JSObject*, QMetaType::Type hint);
    static bool canHandle(QMetaType::Type hint);
};

}

}
#endif
