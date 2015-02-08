

#ifndef MediaQueryEvaluator_h
#define MediaQueryEvaluator_h

#include "PlatformString.h"

namespace WebCore {
class CSSStyleSelector;
class Frame;
class RenderStyle;
class MediaList;
class MediaQueryExp;

class MediaQueryEvaluator : public Noncopyable {
public:
    /** Creates evaluator which evaluates only simple media queries
     *  Evaluator returns true for "all", and returns value of \mediaFeatureResult
     *  for any media features
     */
    MediaQueryEvaluator(bool mediaFeatureResult = false);

    /** Creates evaluator which evaluates only simple media queries
     *  Evaluator  returns true for acceptedMediaType and returns value of \mediafeatureResult
     *  for any media features
     */
    MediaQueryEvaluator(const String& acceptedMediaType, bool mediaFeatureResult = false);
    MediaQueryEvaluator(const char* acceptedMediaType, bool mediaFeatureResult = false);

    /** Creates evaluator which evaluates full media queries
     */
    MediaQueryEvaluator(const String& acceptedMediaType, Frame*, RenderStyle*);

    ~MediaQueryEvaluator();

    bool mediaTypeMatch(const String& mediaTypeToMatch) const;
    bool mediaTypeMatchSpecific(const char* mediaTypeToMatch) const;

    /** Evaluates a list of media queries */
    bool eval(const MediaList*, CSSStyleSelector* = 0) const;

    /** Evaluates media query subexpression, ie "and (media-feature: value)" part */
    bool eval(const MediaQueryExp*) const;

private:
    String m_mediaType;
    Frame* m_frame; // not owned
    RenderStyle* m_style; // not owned
    bool m_expResult;
};

} // namespace
#endif
