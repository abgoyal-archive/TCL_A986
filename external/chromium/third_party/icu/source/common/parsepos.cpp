
#include "unicode/parsepos.h"

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(ParsePosition)

ParsePosition::~ParsePosition() {}

ParsePosition *
ParsePosition::clone() const {
    return new ParsePosition(*this);
}

U_NAMESPACE_END
