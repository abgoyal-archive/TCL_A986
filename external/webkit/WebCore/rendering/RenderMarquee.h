

#ifndef RenderMarquee_h
#define RenderMarquee_h

#include "Length.h"
#include "RenderStyleConstants.h"
#include "Timer.h"

namespace WebCore {

class RenderLayer;

// This class handles the auto-scrolling of layers with overflow: marquee.
class RenderMarquee : public Noncopyable {
public:
    RenderMarquee(RenderLayer*);

    int speed() const { return m_speed; }
    int marqueeSpeed() const;

    EMarqueeDirection reverseDirection() const { return static_cast<EMarqueeDirection>(-direction()); }
    EMarqueeDirection direction() const;

    bool isHorizontal() const;

    int computePosition(EMarqueeDirection, bool stopAtClientEdge);

    void setEnd(int end) { m_end = end; }
    
    void start();
    void suspend();
    void stop();

    void updateMarqueeStyle();
    void updateMarqueePosition();

private:
    void timerFired(Timer<RenderMarquee>*);

    RenderLayer* m_layer;
    int m_currentLoop;
    int m_totalLoops;
    Timer<RenderMarquee> m_timer;
    int m_start;
    int m_end;
    int m_speed;
    Length m_height;
    bool m_reset: 1;
    bool m_suspended : 1;
    bool m_stopped : 1;
    EMarqueeDirection m_direction : 4;
};

} // namespace WebCore

#endif // RenderMarquee_h
