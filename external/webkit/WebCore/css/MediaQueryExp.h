

#ifndef MediaQueryExp_h
#define MediaQueryExp_h

#include "AtomicString.h"
#include "CSSValue.h"
#include "MediaFeatureNames.h"
#include <wtf/RefPtr.h>

namespace WebCore {
class CSSParserValueList;

class MediaQueryExp : public FastAllocBase {
public:
    MediaQueryExp(const AtomicString& mediaFeature, CSSParserValueList* values);
    ~MediaQueryExp();

    AtomicString mediaFeature() const { return m_mediaFeature; }

    CSSValue* value() const { return m_value.get(); }

    bool operator==(const MediaQueryExp& other) const
    {
        return (other.m_mediaFeature == m_mediaFeature)
            && ((!other.m_value && !m_value)
                || (other.m_value && m_value && other.m_value->cssText() == m_value->cssText()));
    }

    bool isViewportDependent() const { return m_mediaFeature == MediaFeatureNames::widthMediaFeature || 
                                              m_mediaFeature == MediaFeatureNames::heightMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::min_widthMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::min_heightMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::max_widthMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::max_heightMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::orientationMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::aspect_ratioMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::min_aspect_ratioMediaFeature ||
                                              m_mediaFeature == MediaFeatureNames::max_aspect_ratioMediaFeature;  }
private:
    AtomicString m_mediaFeature;
    RefPtr<CSSValue> m_value;
};

} // namespace

#endif
