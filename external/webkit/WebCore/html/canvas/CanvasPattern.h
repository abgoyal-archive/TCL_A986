

#ifndef CanvasPattern_h
#define CanvasPattern_h

#include "Pattern.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Image;
    class String;

    typedef int ExceptionCode;

    class CanvasPattern : public RefCounted<CanvasPattern> {
    public:
        static void parseRepetitionType(const String&, bool& repeatX, bool& repeatY, ExceptionCode&);

        static PassRefPtr<CanvasPattern> create(Image* image, bool repeatX, bool repeatY, bool originClean)
        {
            return adoptRef(new CanvasPattern(image, repeatX, repeatY, originClean));
        }

        Pattern* pattern() const { return m_pattern.get(); }

        bool originClean() const { return m_originClean; }

    private:
        CanvasPattern(Image*, bool repeatX, bool repeatY, bool originClean);

        RefPtr<Pattern> m_pattern;
        bool m_originClean;
    };

} // namespace WebCore

#endif
