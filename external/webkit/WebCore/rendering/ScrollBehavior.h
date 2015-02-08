

#ifndef ScrollBehavior_h
#define ScrollBehavior_h

namespace WebCore {

enum ScrollBehavior {
    noScroll,
    alignCenter,
    alignTop,
    alignBottom, 
    alignLeft,
    alignRight,
    alignToClosestEdge
};

struct ScrollAlignment {
    static ScrollBehavior getVisibleBehavior(const ScrollAlignment& s) { return s.m_rectVisible; }
    static ScrollBehavior getPartialBehavior(const ScrollAlignment& s) { return s.m_rectPartial; }
    static ScrollBehavior getHiddenBehavior(const ScrollAlignment& s) { return s.m_rectHidden; }

    static const ScrollAlignment alignCenterIfNeeded;
    static const ScrollAlignment alignToEdgeIfNeeded;
    static const ScrollAlignment alignCenterAlways;
    static const ScrollAlignment alignTopAlways;
    static const ScrollAlignment alignBottomAlways;

    ScrollBehavior m_rectVisible;
    ScrollBehavior m_rectHidden;
    ScrollBehavior m_rectPartial;
};


}; // namespace WebCore

#endif // ScrollBehavior_h
