

#ifndef Geoposition_h
#define Geoposition_h

#include "Coordinates.h"
#include "Event.h"
#include "PlatformString.h"
#include <wtf/RefCounted.h>

namespace WebCore {

typedef int ExceptionCode;

class Geoposition : public RefCounted<Geoposition> {
public:
    static PassRefPtr<Geoposition> create(PassRefPtr<Coordinates> coordinates, DOMTimeStamp timestamp)
    {
        return adoptRef(new Geoposition(coordinates, timestamp));
    }

    DOMTimeStamp timestamp() const { return m_timestamp; }
    Coordinates* coords() const { return m_coordinates.get(); }
    
private:
    Geoposition(PassRefPtr<Coordinates> coordinates, DOMTimeStamp timestamp)
        : m_coordinates(coordinates)
        , m_timestamp(timestamp)
    {
    }

    RefPtr<Coordinates> m_coordinates;
    DOMTimeStamp m_timestamp;
};
    
} // namespace WebCore

#endif // Geoposition_h
