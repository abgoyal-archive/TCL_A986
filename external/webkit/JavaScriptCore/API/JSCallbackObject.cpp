

#include "config.h"
#include "JSCallbackObject.h"

#include "Collector.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSCallbackObject<JSObject>);
ASSERT_CLASS_FITS_IN_CELL(JSCallbackObject<JSGlobalObject>);

// Define the two types of JSCallbackObjects we support.
template <> const ClassInfo JSCallbackObject<JSObject>::info = { "CallbackObject", 0, 0, 0 };
template <> const ClassInfo JSCallbackObject<JSGlobalObject>::info = { "CallbackGlobalObject", 0, 0, 0 };

} // namespace JSC
