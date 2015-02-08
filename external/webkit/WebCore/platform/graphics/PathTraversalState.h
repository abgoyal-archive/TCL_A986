

#ifndef PathTraversalState_h
#define PathTraversalState_h

#include "FloatPoint.h"
#include <wtf/Vector.h>

namespace WebCore {
    
    class Path;
    
    class PathTraversalState {
    public:
        enum PathTraversalAction {
            TraversalTotalLength,
            TraversalPointAtLength,
            TraversalSegmentAtLength,
            TraversalNormalAngleAtLength
        };
        
        PathTraversalState(PathTraversalAction);
        
        float closeSubpath();
        float moveTo(const FloatPoint&);
        float lineTo(const FloatPoint&);
        float quadraticBezierTo(const FloatPoint& newControl, const FloatPoint& newEnd);
        float cubicBezierTo(const FloatPoint& newControl1, const FloatPoint& newControl2, const FloatPoint& newEnd);
        
    public:
        PathTraversalAction m_action;
        bool m_success;
        
        FloatPoint m_current;
        FloatPoint m_start;
        FloatPoint m_control1;
        FloatPoint m_control2;
        
        float m_totalLength;
        unsigned m_segmentIndex;
        float m_desiredLength;
        
        // For normal calculations
        FloatPoint m_previous;
        float m_normalAngle; // degrees
    };    
}

#endif
