
 
// this is a hacky way of doing it, but we don't
// currently have info on which platform we're being
// compiled for directly.
#if __WXMAC__
#include "mac/carbon/fontprops.cpp"
#elif __WXMSW__
#include "win/fontprops.cpp"
#else
#include "gtk/fontprops.cpp"
#endif
