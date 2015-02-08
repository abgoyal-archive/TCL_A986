

#ifndef TOUCHLIST_H_
#define TOUCHLIST_H_

#if ENABLE(TOUCH_EVENTS)

#include "Touch.h"
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

class TouchList : public RefCounted<TouchList> {
public:
    static PassRefPtr<TouchList> create()
    {
        return adoptRef(new TouchList);
    }

    unsigned length() const { return m_values.size(); }

    Touch* item(unsigned);

    void append(const PassRefPtr<Touch> touch) { m_values.append(touch); }

private:
    TouchList() {}

    Vector<RefPtr<Touch> > m_values;
};

} // namespace WebCore

#endif // ENABLE(TOUCH_EVENTS)

#endif /* TOUCHLIST_H_ */
