

#ifndef PluginTimer_H
#define PluginTimer_H

#include "Timer.h"
#include "npapi.h"

namespace WebCore {

    class PluginTimerList;

    class PluginTimer : public TimerBase {
    public:
        PluginTimer(PluginTimer** list, NPP instance, bool repeat,
                    void (*proc)(NPP npp, uint32 timerID));
        virtual ~PluginTimer();
    
        uint32 timerID() const { return m_timerID; }

        void unschedule() { m_unscheduled = true; }

        static PluginTimer* Find(PluginTimer* list, uint32 timerID);

    private:
        // override from TimerBase
        virtual void fired();
        
        PluginTimer* next() const { return m_next; }
        friend class PluginTimerList;

        PluginTimer**   m_list;
        PluginTimer*    m_prev;
        PluginTimer*    m_next;
        NPP             m_instance;
        void            (*m_timerFunc)(NPP, uint32);
        uint32          m_timerID;
        bool            m_repeat;
        bool            m_unscheduled;
    };
    
    class PluginTimerList {
    public:
        PluginTimerList() : m_list(0) {}
        ~PluginTimerList();
        
        uint32 schedule(NPP instance, uint32 interval, bool repeat,
                        void (*proc)(NPP npp, uint32 timerID));
        void unschedule(NPP instance, uint32 timerID);
        
    private:
        PluginTimer* m_list;
    };

} // namespace WebCore

#endif
