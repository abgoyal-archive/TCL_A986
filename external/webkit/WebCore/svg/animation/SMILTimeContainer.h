

#ifndef SMILTimeContainer_H
#define SMILTimeContainer_H

#if ENABLE(SVG)

#include "PlatformString.h"
#include "SMILTime.h"
#include "StringHash.h"
#include "Timer.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class SVGElement;
    class SVGSMILElement;
    class SVGSVGElement;

    class SMILTimeContainer : public RefCounted<SMILTimeContainer>  {
    public:
        static PassRefPtr<SMILTimeContainer> create(SVGSVGElement* owner) { return adoptRef(new SMILTimeContainer(owner)); } 
    
        void schedule(SVGSMILElement*);
        void unschedule(SVGSMILElement*);
        
        SMILTime elapsed() const;

        bool isActive() const;
        bool isPaused() const;
        
        void begin();
        void pause();
        void resume();
        
        void setDocumentOrderIndexesDirty() { m_documentOrderIndexesDirty = true; }

        // Move to a specific time. Only used for DRT testing purposes.
        void sampleAnimationAtTime(const String& elementId, double seconds);

    private:
        SMILTimeContainer(SVGSVGElement* owner);
        
        void timerFired(Timer<SMILTimeContainer>*);
        void startTimer(SMILTime fireTime, SMILTime minimumDelay = 0);
        void updateAnimations(SMILTime elapsed);
        
        void updateDocumentOrderIndexes();
        void sortByPriority(Vector<SVGSMILElement*>& smilElements, SMILTime elapsed);
        
        typedef pair<SVGElement*, String> ElementAttributePair;
        String baseValueFor(ElementAttributePair);
        
        double m_beginTime;
        double m_pauseTime;
        double m_accumulatedPauseTime;
        double m_nextManualSampleTime;
        String m_nextSamplingTarget;

        bool m_documentOrderIndexesDirty;
        
        Timer<SMILTimeContainer> m_timer;

        typedef HashSet<SVGSMILElement*> TimingElementSet;
        TimingElementSet m_scheduledAnimations;
        
        typedef HashMap<ElementAttributePair, String> BaseValueMap;
        BaseValueMap m_savedBaseValues;

        SVGSVGElement* m_ownerSVGElement;
    };
}

#endif
#endif
