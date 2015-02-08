
#ifndef setsignal_h
#define setsignal_h

RETSIGTYPE (*setsignal(int, RETSIGTYPE (*)(int)))(int);
#endif
