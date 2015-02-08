
#include "unicode/tmunit.h"

#if !UCONFIG_NO_FORMATTING

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(TimeUnit)


TimeUnit* U_EXPORT2 
TimeUnit::createInstance(TimeUnit::UTimeUnitFields timeUnitField, 
                         UErrorCode& status) {
    if (U_FAILURE(status)) {
        return NULL;
    }
    if (timeUnitField < 0 || timeUnitField >= UTIMEUNIT_FIELD_COUNT) {
        status = U_ILLEGAL_ARGUMENT_ERROR;
        return NULL;
    }
    return new TimeUnit(timeUnitField);
}


TimeUnit::TimeUnit(TimeUnit::UTimeUnitFields timeUnitField) {
    fTimeUnitField = timeUnitField;
}


TimeUnit::TimeUnit(const TimeUnit& other) 
:   MeasureUnit(other) {
    *this = other;
}


UObject* 
TimeUnit::clone() const {
    return new TimeUnit(*this);
}


TimeUnit&
TimeUnit::operator=(const TimeUnit& other) {
    if (this == &other) {
        return *this;
    }
    fTimeUnitField = other.fTimeUnitField;
    return *this;
}


UBool 
TimeUnit::operator==(const UObject& other) const {
    return (other.getDynamicClassID() == TimeUnit::getStaticClassID() 
            && fTimeUnitField == ((TimeUnit*)&other)->fTimeUnitField);
}


TimeUnit::UTimeUnitFields
TimeUnit::getTimeUnitField() const {
    return fTimeUnitField;
}


TimeUnit::~TimeUnit() {
}


U_NAMESPACE_END

#endif
