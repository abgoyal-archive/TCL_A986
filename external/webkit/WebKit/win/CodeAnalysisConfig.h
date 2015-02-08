

#ifndef CodeAnalysisConfig_H
#define CodeAnalysisConfig_H

// warning C6011: dereferencing NULL pointer <name>
// warning C6031: return value ignored: <function> could return unexpected value
// warning C6211: Leaking memory due to an exception. Consider using a local catch block to clean up memory.
// warning C6255: _alloca indicates failure by raising a stack overflow exception. Consider using _alloca_s instead
// warning C6387: <argument> may be <value>: this does not adhere to the specification for the function <function name>: Lines: x, y
#pragma warning(disable: 6011 6031 6211 6255 6387)

#endif
