


#undef _begin_code_h

/* Reset structure packing at previous byte alignment */
#if defined(_MSC_VER) || defined(__MWERKS__) || defined(__WATCOMC__)  || defined(__BORLANDC__)
#ifdef __BORLANDC__
#pragma nopackwarning
#endif
#if (defined(__MWERKS__) && defined(__MACOS__))
#pragma options align=reset
#pragma enumsalwaysint reset
#else
#pragma pack(pop)
#endif
#endif /* Compiler needs structure packing set */

