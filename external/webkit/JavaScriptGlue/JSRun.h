

#ifndef JSRun_h
#define JSRun_h

#include "JSBase.h"
#include "JSUtils.h"

class JSGlueGlobalObject : public JSGlobalObject {
    public:
        JSGlueGlobalObject(PassRefPtr<Structure>, JSFlags = kJSFlagNone);

        JSFlags Flags() const { return d()->flags; }
        Structure* userObjectStructure() const { return d()->userObjectStructure.get(); }

    private:
        struct Data : JSGlobalObjectData {
            Data()
                : JSGlobalObjectData(destroyData)
            {
            }
            
            RefPtr<Structure> userObjectStructure;
            JSFlags flags;
        };

        static void destroyData(void*);

        Data* d() const { return static_cast<Data*>(JSGlobalObject::d()); }
};

class JSRun : public JSBase {
    public:
        JSRun(CFStringRef source, JSFlags inFlags);
        virtual ~JSRun();

        UString GetSource() const;
        JSGlobalObject* GlobalObject() const;
        Completion Evaluate();
        bool CheckSyntax();
        JSFlags Flags() const;
    private:
        UString fSource;
        ProtectedPtr<JSGlobalObject> fGlobalObject;
        JSFlags fFlags;
};

#endif
