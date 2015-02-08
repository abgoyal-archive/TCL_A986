

#include "config.h"
#include "HTTPHeaderMap.h"

#include <memory>
#include <utility>

using namespace std;

namespace WebCore {

auto_ptr<CrossThreadHTTPHeaderMapData> HTTPHeaderMap::copyData() const
{
    auto_ptr<CrossThreadHTTPHeaderMapData> data(new CrossThreadHTTPHeaderMapData());
    data->reserveInitialCapacity(size());

    HTTPHeaderMap::const_iterator end_it = end();
    for (HTTPHeaderMap::const_iterator it = begin(); it != end_it; ++it) {
        data->append(make_pair(it->first.string().crossThreadString(), it->second.crossThreadString()));
    }
    return data;
}

void HTTPHeaderMap::adopt(auto_ptr<CrossThreadHTTPHeaderMapData> data)
{
    clear();
    size_t dataSize = data->size();
    for (size_t index = 0; index < dataSize; ++index) {
        pair<String, String>& header = (*data)[index];
        set(header.first, header.second);
    }
}
    
// Adapter that allows the HashMap to take C strings as keys.
struct CaseFoldingCStringTranslator {
    static unsigned hash(const char* cString)
    {
        return CaseFoldingHash::hash(cString, strlen(cString));
    }
    
    static bool equal(const AtomicString& key, const char* cString)
    {
        return equalIgnoringCase(key, cString);
    }
    
    static void translate(AtomicString& location, const char* cString, unsigned /*hash*/)
    {
        location = AtomicString(cString);
    }
};

String HTTPHeaderMap::get(const char* name) const
{
    const_iterator i = find<const char*, CaseFoldingCStringTranslator>(name);
    if (i == end())
        return String();
    return i->second;
}
    
bool HTTPHeaderMap::contains(const char* name) const
{
    return find<const char*, CaseFoldingCStringTranslator>(name) != end();
}

pair<HTTPHeaderMap::iterator, bool> HTTPHeaderMap::add(const char* name, const String& value)
{
    return HashMap<AtomicString, String, CaseFoldingHash>::add<const char*, CaseFoldingCStringTranslator>(name, value);
}

} // namespace WebCore
