

#ifndef HTTPHeaderMap_h
#define HTTPHeaderMap_h

#include "AtomicString.h"
#include "AtomicStringHash.h"
#include "StringHash.h"
#include <memory>
#include <utility>
#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace WebCore {

    typedef Vector<std::pair<String, String> > CrossThreadHTTPHeaderMapData;

    class HTTPHeaderMap : public HashMap<AtomicString, String, CaseFoldingHash> {
    public:
        // Gets a copy of the data suitable for passing to another thread.
        std::auto_ptr<CrossThreadHTTPHeaderMapData> copyData() const;

        void adopt(std::auto_ptr<CrossThreadHTTPHeaderMapData>);
        
        String get(const AtomicString& name) const
        {
            return HashMap<AtomicString, String, CaseFoldingHash>::get(name);
        }

        pair<iterator, bool> add(const AtomicString& name, const String& value)
        {
            return HashMap<AtomicString, String, CaseFoldingHash>::add(name, value);
        }

        // Alternate accessors that are faster than converting the char* to AtomicString first.
        bool contains(const char*) const;
        String get(const char*) const;
        pair<iterator, bool> add(const char* name, const String& value);
        
    };

} // namespace WebCore

#endif // HTTPHeaderMap_h
