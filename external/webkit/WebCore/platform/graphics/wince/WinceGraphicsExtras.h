

#ifndef WinceGraphicsExtras_h
#define WinceGraphicsExtras_h

// This file is used to contain small utilities used by WINCE graphics code.

namespace WebCore {
    // Always round to same direction. 0.5 is rounded to 1,
    // and -0.5 (0.5 - 1) is rounded to 0 (1 - 1), so that it
    // is consistent when transformation shifts.
    static inline int stableRound(double d)
    {
        if (d > 0)
            return static_cast<int>(d + 0.5);

        int i = static_cast<int>(d);
        return i - d > 0.5 ? i - 1 : i;
    }
}

#endif WinceGraphicsExtras_h
