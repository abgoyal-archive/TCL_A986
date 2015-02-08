

#ifndef ExceptionBase_h
#define ExceptionBase_h

#include "ExceptionCode.h"
#include "PlatformString.h"
#include <wtf/RefCounted.h>

namespace WebCore {

    class ExceptionBase : public RefCounted<ExceptionBase> {
    public:
        unsigned short code() const { return m_code; }
        String name() const { return m_name; }
        String message() const { return m_message; }
        String description() const { return m_description; }

        String toString() const;

    protected:
        ExceptionBase(const ExceptionCodeDescription&);

    private:
        unsigned short m_code;
        String m_name;
        String m_message;
        String m_description;
    };

} // namespace WebCore

#endif // ExceptionBase_h
