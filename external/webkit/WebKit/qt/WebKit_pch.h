



#if defined __cplusplus

#if defined(_WIN32) && !defined(_WIN32_WCE)
#define _CRT_RAND_S
#define _WIN32_WINNT 0x0500
#include <windows.h>
#endif

#include "../../JavaScriptCore/config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <assert.h>
#include <ctype.h>
#include <limits.h>

#include <wtf/HashTable.h>
#include <wtf/Vector.h>
#include <wtf/ListHashSet.h>
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/HashTraits.h>
#include <wtf/HashIterators.h>
#include <wtf/TCPageMap.h>
#include <wtf/Assertions.h>
#include <wtf/HashCountedSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Platform.h>
#include <wtf/RefPtr.h>
#include <wtf/VectorTraits.h>
#include <wtf/MathExtras.h>
#include <wtf/HashFunctions.h>
#include <wtf/OwnPtr.h>
#include <wtf/OwnArrayPtr.h>
#include <wtf/ListRefPtr.h>
#include <wtf/FastMalloc.h>
#include <wtf/TCSystemAlloc.h>
#include <wtf/StringExtras.h>
#include <wtf/Noncopyable.h>
#include <wtf/Forward.h>
#include <wtf/UnusedParam.h>
#include <wtf/AlwaysInline.h>
#include <wtf/GetPtr.h>
#endif
