

#ifndef RegExpConstructor_h
#define RegExpConstructor_h

#include "InternalFunction.h"
#include "RegExp.h"
#include <wtf/OwnPtr.h>

namespace JSC {

    class RegExp;
    class RegExpPrototype;
    struct RegExpConstructorPrivate;

    struct RegExpConstructorPrivate : FastAllocBase {
        // Global search cache / settings
        RegExpConstructorPrivate()
            : lastNumSubPatterns(0)
            , multiline(false)
            , lastOvectorIndex(0)
        {
        }

        const Vector<int, 32>& lastOvector() const { return ovector[lastOvectorIndex]; }
        Vector<int, 32>& lastOvector() { return ovector[lastOvectorIndex]; }
        Vector<int, 32>& tempOvector() { return ovector[lastOvectorIndex ? 0 : 1]; }
        void changeLastOvector() { lastOvectorIndex = lastOvectorIndex ? 0 : 1; }

        UString input;
        UString lastInput;
        Vector<int, 32> ovector[2];
        unsigned lastNumSubPatterns : 30;
        bool multiline : 1;
        unsigned lastOvectorIndex : 1;
    };

    class RegExpConstructor : public InternalFunction {
    public:
        RegExpConstructor(ExecState*, NonNullPassRefPtr<Structure>, RegExpPrototype*);

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

        virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
        virtual bool getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        static const ClassInfo info;

        void performMatch(RegExp*, const UString&, int startOffset, int& position, int& length, int** ovector = 0);
        JSObject* arrayOfMatches(ExecState*) const;

        void setInput(const UString&);
        const UString& input() const;

        void setMultiline(bool);
        bool multiline() const;

        JSValue getBackref(ExecState*, unsigned) const;
        JSValue getLastParen(ExecState*) const;
        JSValue getLeftContext(ExecState*) const;
        JSValue getRightContext(ExecState*) const;

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | ImplementsHasInstance | InternalFunction::StructureFlags;

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);

        virtual const ClassInfo* classInfo() const { return &info; }

        OwnPtr<RegExpConstructorPrivate> d;
    };

    RegExpConstructor* asRegExpConstructor(JSValue);

    JSObject* constructRegExp(ExecState*, const ArgList&);

    inline RegExpConstructor* asRegExpConstructor(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&RegExpConstructor::info));
        return static_cast<RegExpConstructor*>(asObject(value));
    }

    /* 
      To facilitate result caching, exec(), test(), match(), search(), and replace() dipatch regular
      expression matching through the performMatch function. We use cached results to calculate, 
      e.g., RegExp.lastMatch and RegExp.leftParen.
    */
    inline void RegExpConstructor::performMatch(RegExp* r, const UString& s, int startOffset, int& position, int& length, int** ovector)
    {
        position = r->match(s, startOffset, &d->tempOvector());

        if (ovector)
            *ovector = d->tempOvector().data();

        if (position != -1) {
            ASSERT(!d->tempOvector().isEmpty());

            length = d->tempOvector()[1] - d->tempOvector()[0];

            d->input = s;
            d->lastInput = s;
            d->changeLastOvector();
            d->lastNumSubPatterns = r->numSubpatterns();
        }
    }

} // namespace JSC

#endif // RegExpConstructor_h
