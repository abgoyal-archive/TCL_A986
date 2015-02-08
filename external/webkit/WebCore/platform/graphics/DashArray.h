

#ifndef DashArray_h
#define DashArray_h

#include <wtf/Vector.h>

#if PLATFORM(CG)
typedef Vector<CGFloat> DashArray;
#elif PLATFORM(CAIRO)
typedef Vector<double> DashArray;
#else
typedef Vector<float> DashArray;
#endif

#endif // DashArray_h
