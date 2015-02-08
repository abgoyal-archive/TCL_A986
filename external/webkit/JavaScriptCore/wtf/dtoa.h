

#ifndef WTF_dtoa_h
#define WTF_dtoa_h

namespace WTF {
    class Mutex;
}

namespace WTF {

    extern WTF::Mutex* s_dtoaP5Mutex;

    double strtod(const char* s00, char** se);

    typedef char DtoaBuffer[80];
    void dtoa(DtoaBuffer result, double d, int ndigits, int* decpt, int* sign, char** rve);

    // dtoa() for ECMA-262 'ToString Applied to the Number Type.'
    // The *resultLength will have the length of the resultant string in bufer.
    // The resultant string isn't terminated by 0.
    void doubleToStringInJavaScriptFormat(double, DtoaBuffer, unsigned* resultLength);

} // namespace WTF

using WTF::DtoaBuffer;
using WTF::doubleToStringInJavaScriptFormat;

#endif // WTF_dtoa_h
