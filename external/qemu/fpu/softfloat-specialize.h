


#if defined(TARGET_MIPS) || defined(TARGET_HPPA)
#define SNAN_BIT_IS_ONE		1
#else
#define SNAN_BIT_IS_ONE		0
#endif


void float_raise( int8 flags STATUS_PARAM )
{
    STATUS(float_exception_flags) |= flags;
}

typedef struct {
    flag sign;
    bits64 high, low;
} commonNaNT;

#if defined(TARGET_SPARC)
#define float32_default_nan make_float32(0x7FFFFFFF)
#elif defined(TARGET_POWERPC) || defined(TARGET_ARM) || defined(TARGET_ALPHA)
#define float32_default_nan make_float32(0x7FC00000)
#elif defined(TARGET_HPPA)
#define float32_default_nan make_float32(0x7FA00000)
#elif SNAN_BIT_IS_ONE
#define float32_default_nan make_float32(0x7FBFFFFF)
#else
#define float32_default_nan make_float32(0xFFC00000)
#endif


int float32_is_nan( float32 a_ )
{
    uint32_t a = float32_val(a_);
#if SNAN_BIT_IS_ONE
    return ( ( ( a>>22 ) & 0x1FF ) == 0x1FE ) && ( a & 0x003FFFFF );
#else
    return ( 0xFF800000 <= (bits32) ( a<<1 ) );
#endif
}


int float32_is_signaling_nan( float32 a_ )
{
    uint32_t a = float32_val(a_);
#if SNAN_BIT_IS_ONE
    return ( 0xFF800000 <= (bits32) ( a<<1 ) );
#else
    return ( ( ( a>>22 ) & 0x1FF ) == 0x1FE ) && ( a & 0x003FFFFF );
#endif
}


static commonNaNT float32ToCommonNaN( float32 a STATUS_PARAM )
{
    commonNaNT z;

    if ( float32_is_signaling_nan( a ) ) float_raise( float_flag_invalid STATUS_VAR );
    z.sign = float32_val(a)>>31;
    z.low = 0;
    z.high = ( (bits64) float32_val(a) )<<41;
    return z;
}


static float32 commonNaNToFloat32( commonNaNT a )
{
    bits32 mantissa = a.high>>41;
    if ( mantissa )
        return make_float32(
            ( ( (bits32) a.sign )<<31 ) | 0x7F800000 | ( a.high>>41 ) );
    else
        return float32_default_nan;
}


static float32 propagateFloat32NaN( float32 a, float32 b STATUS_PARAM)
{
    flag aIsNaN, aIsSignalingNaN, bIsNaN, bIsSignalingNaN;
    bits32 av, bv, res;

    if ( STATUS(default_nan_mode) )
        return float32_default_nan;

    aIsNaN = float32_is_nan( a );
    aIsSignalingNaN = float32_is_signaling_nan( a );
    bIsNaN = float32_is_nan( b );
    bIsSignalingNaN = float32_is_signaling_nan( b );
    av = float32_val(a);
    bv = float32_val(b);
#if SNAN_BIT_IS_ONE
    av &= ~0x00400000;
    bv &= ~0x00400000;
#else
    av |= 0x00400000;
    bv |= 0x00400000;
#endif
    if ( aIsSignalingNaN | bIsSignalingNaN ) float_raise( float_flag_invalid STATUS_VAR);
    if ( aIsSignalingNaN ) {
        if ( bIsSignalingNaN ) goto returnLargerSignificand;
        res = bIsNaN ? bv : av;
    }
    else if ( aIsNaN ) {
        if ( bIsSignalingNaN || ! bIsNaN )
            res = av;
        else {
 returnLargerSignificand:
            if ( (bits32) ( av<<1 ) < (bits32) ( bv<<1 ) )
                res = bv;
            else if ( (bits32) ( bv<<1 ) < (bits32) ( av<<1 ) )
                res = av;
            else
                res = ( av < bv ) ? av : bv;
        }
    }
    else {
        res = bv;
    }
    return make_float32(res);
}

#if defined(TARGET_SPARC)
#define float64_default_nan make_float64(LIT64( 0x7FFFFFFFFFFFFFFF ))
#elif defined(TARGET_POWERPC) || defined(TARGET_ARM) || defined(TARGET_ALPHA)
#define float64_default_nan make_float64(LIT64( 0x7FF8000000000000 ))
#elif defined(TARGET_HPPA)
#define float64_default_nan make_float64(LIT64( 0x7FF4000000000000 ))
#elif SNAN_BIT_IS_ONE
#define float64_default_nan make_float64(LIT64( 0x7FF7FFFFFFFFFFFF ))
#else
#define float64_default_nan make_float64(LIT64( 0xFFF8000000000000 ))
#endif


int float64_is_nan( float64 a_ )
{
    bits64 a = float64_val(a_);
#if SNAN_BIT_IS_ONE
    return
           ( ( ( a>>51 ) & 0xFFF ) == 0xFFE )
        && ( a & LIT64( 0x0007FFFFFFFFFFFF ) );
#else
    return ( LIT64( 0xFFF0000000000000 ) <= (bits64) ( a<<1 ) );
#endif
}


int float64_is_signaling_nan( float64 a_ )
{
    bits64 a = float64_val(a_);
#if SNAN_BIT_IS_ONE
    return ( LIT64( 0xFFF0000000000000 ) <= (bits64) ( a<<1 ) );
#else
    return
           ( ( ( a>>51 ) & 0xFFF ) == 0xFFE )
        && ( a & LIT64( 0x0007FFFFFFFFFFFF ) );
#endif
}


static commonNaNT float64ToCommonNaN( float64 a STATUS_PARAM)
{
    commonNaNT z;

    if ( float64_is_signaling_nan( a ) ) float_raise( float_flag_invalid STATUS_VAR);
    z.sign = float64_val(a)>>63;
    z.low = 0;
    z.high = float64_val(a)<<12;
    return z;
}


static float64 commonNaNToFloat64( commonNaNT a )
{
    bits64 mantissa = a.high>>12;

    if ( mantissa )
        return make_float64(
              ( ( (bits64) a.sign )<<63 )
            | LIT64( 0x7FF0000000000000 )
            | ( a.high>>12 ));
    else
        return float64_default_nan;
}


static float64 propagateFloat64NaN( float64 a, float64 b STATUS_PARAM)
{
    flag aIsNaN, aIsSignalingNaN, bIsNaN, bIsSignalingNaN;
    bits64 av, bv, res;

    if ( STATUS(default_nan_mode) )
        return float64_default_nan;

    aIsNaN = float64_is_nan( a );
    aIsSignalingNaN = float64_is_signaling_nan( a );
    bIsNaN = float64_is_nan( b );
    bIsSignalingNaN = float64_is_signaling_nan( b );
    av = float64_val(a);
    bv = float64_val(b);
#if SNAN_BIT_IS_ONE
    av &= ~LIT64( 0x0008000000000000 );
    bv &= ~LIT64( 0x0008000000000000 );
#else
    av |= LIT64( 0x0008000000000000 );
    bv |= LIT64( 0x0008000000000000 );
#endif
    if ( aIsSignalingNaN | bIsSignalingNaN ) float_raise( float_flag_invalid STATUS_VAR);
    if ( aIsSignalingNaN ) {
        if ( bIsSignalingNaN ) goto returnLargerSignificand;
        res = bIsNaN ? bv : av;
    }
    else if ( aIsNaN ) {
        if ( bIsSignalingNaN || ! bIsNaN )
            res = av;
        else {
 returnLargerSignificand:
            if ( (bits64) ( av<<1 ) < (bits64) ( bv<<1 ) )
                res = bv;
            else if ( (bits64) ( bv<<1 ) < (bits64) ( av<<1 ) )
                res = av;
            else
                res = ( av < bv ) ? av : bv;
        }
    }
    else {
        res = bv;
    }
    return make_float64(res);
}

#ifdef FLOATX80

#if SNAN_BIT_IS_ONE
#define floatx80_default_nan_high 0x7FFF
#define floatx80_default_nan_low  LIT64( 0xBFFFFFFFFFFFFFFF )
#else
#define floatx80_default_nan_high 0xFFFF
#define floatx80_default_nan_low  LIT64( 0xC000000000000000 )
#endif


int floatx80_is_nan( floatx80 a )
{
#if SNAN_BIT_IS_ONE
    bits64 aLow;

    aLow = a.low & ~ LIT64( 0x4000000000000000 );
    return
           ( ( a.high & 0x7FFF ) == 0x7FFF )
        && (bits64) ( aLow<<1 )
        && ( a.low == aLow );
#else
    return ( ( a.high & 0x7FFF ) == 0x7FFF ) && (bits64) ( a.low<<1 );
#endif
}


int floatx80_is_signaling_nan( floatx80 a )
{
#if SNAN_BIT_IS_ONE
    return ( ( a.high & 0x7FFF ) == 0x7FFF ) && (bits64) ( a.low<<1 );
#else
    bits64 aLow;

    aLow = a.low & ~ LIT64( 0x4000000000000000 );
    return
           ( ( a.high & 0x7FFF ) == 0x7FFF )
        && (bits64) ( aLow<<1 )
        && ( a.low == aLow );
#endif
}


static commonNaNT floatx80ToCommonNaN( floatx80 a STATUS_PARAM)
{
    commonNaNT z;

    if ( floatx80_is_signaling_nan( a ) ) float_raise( float_flag_invalid STATUS_VAR);
    z.sign = a.high>>15;
    z.low = 0;
    z.high = a.low;
    return z;
}


static floatx80 commonNaNToFloatx80( commonNaNT a )
{
    floatx80 z;

    if (a.high)
        z.low = a.high;
    else
        z.low = floatx80_default_nan_low;
    z.high = ( ( (bits16) a.sign )<<15 ) | 0x7FFF;
    return z;
}


static floatx80 propagateFloatx80NaN( floatx80 a, floatx80 b STATUS_PARAM)
{
    flag aIsNaN, aIsSignalingNaN, bIsNaN, bIsSignalingNaN;

    if ( STATUS(default_nan_mode) ) {
        a.low = floatx80_default_nan_low;
        a.high = floatx80_default_nan_high;
        return a;
    }

    aIsNaN = floatx80_is_nan( a );
    aIsSignalingNaN = floatx80_is_signaling_nan( a );
    bIsNaN = floatx80_is_nan( b );
    bIsSignalingNaN = floatx80_is_signaling_nan( b );
#if SNAN_BIT_IS_ONE
    a.low &= ~LIT64( 0xC000000000000000 );
    b.low &= ~LIT64( 0xC000000000000000 );
#else
    a.low |= LIT64( 0xC000000000000000 );
    b.low |= LIT64( 0xC000000000000000 );
#endif
    if ( aIsSignalingNaN | bIsSignalingNaN ) float_raise( float_flag_invalid STATUS_VAR);
    if ( aIsSignalingNaN ) {
        if ( bIsSignalingNaN ) goto returnLargerSignificand;
        return bIsNaN ? b : a;
    }
    else if ( aIsNaN ) {
        if ( bIsSignalingNaN || ! bIsNaN ) return a;
 returnLargerSignificand:
        if ( a.low < b.low ) return b;
        if ( b.low < a.low ) return a;
        return ( a.high < b.high ) ? a : b;
    }
    else {
        return b;
    }
}

#endif

#ifdef FLOAT128

#if SNAN_BIT_IS_ONE
#define float128_default_nan_high LIT64( 0x7FFF7FFFFFFFFFFF )
#define float128_default_nan_low  LIT64( 0xFFFFFFFFFFFFFFFF )
#else
#define float128_default_nan_high LIT64( 0xFFFF800000000000 )
#define float128_default_nan_low  LIT64( 0x0000000000000000 )
#endif


int float128_is_nan( float128 a )
{
#if SNAN_BIT_IS_ONE
    return
           ( ( ( a.high>>47 ) & 0xFFFF ) == 0xFFFE )
        && ( a.low || ( a.high & LIT64( 0x00007FFFFFFFFFFF ) ) );
#else
    return
           ( LIT64( 0xFFFE000000000000 ) <= (bits64) ( a.high<<1 ) )
        && ( a.low || ( a.high & LIT64( 0x0000FFFFFFFFFFFF ) ) );
#endif
}


int float128_is_signaling_nan( float128 a )
{
#if SNAN_BIT_IS_ONE
    return
           ( LIT64( 0xFFFE000000000000 ) <= (bits64) ( a.high<<1 ) )
        && ( a.low || ( a.high & LIT64( 0x0000FFFFFFFFFFFF ) ) );
#else
    return
           ( ( ( a.high>>47 ) & 0xFFFF ) == 0xFFFE )
        && ( a.low || ( a.high & LIT64( 0x00007FFFFFFFFFFF ) ) );
#endif
}


static commonNaNT float128ToCommonNaN( float128 a STATUS_PARAM)
{
    commonNaNT z;

    if ( float128_is_signaling_nan( a ) ) float_raise( float_flag_invalid STATUS_VAR);
    z.sign = a.high>>63;
    shortShift128Left( a.high, a.low, 16, &z.high, &z.low );
    return z;
}


static float128 commonNaNToFloat128( commonNaNT a )
{
    float128 z;

    shift128Right( a.high, a.low, 16, &z.high, &z.low );
    z.high |= ( ( (bits64) a.sign )<<63 ) | LIT64( 0x7FFF000000000000 );
    return z;
}


static float128 propagateFloat128NaN( float128 a, float128 b STATUS_PARAM)
{
    flag aIsNaN, aIsSignalingNaN, bIsNaN, bIsSignalingNaN;

    if ( STATUS(default_nan_mode) ) {
        a.low = float128_default_nan_low;
        a.high = float128_default_nan_high;
        return a;
    }

    aIsNaN = float128_is_nan( a );
    aIsSignalingNaN = float128_is_signaling_nan( a );
    bIsNaN = float128_is_nan( b );
    bIsSignalingNaN = float128_is_signaling_nan( b );
#if SNAN_BIT_IS_ONE
    a.high &= ~LIT64( 0x0000800000000000 );
    b.high &= ~LIT64( 0x0000800000000000 );
#else
    a.high |= LIT64( 0x0000800000000000 );
    b.high |= LIT64( 0x0000800000000000 );
#endif
    if ( aIsSignalingNaN | bIsSignalingNaN ) float_raise( float_flag_invalid STATUS_VAR);
    if ( aIsSignalingNaN ) {
        if ( bIsSignalingNaN ) goto returnLargerSignificand;
        return bIsNaN ? b : a;
    }
    else if ( aIsNaN ) {
        if ( bIsSignalingNaN || ! bIsNaN ) return a;
 returnLargerSignificand:
        if ( lt128( a.high<<1, a.low, b.high<<1, b.low ) ) return b;
        if ( lt128( b.high<<1, b.low, a.high<<1, a.low ) ) return a;
        return ( a.high < b.high ) ? a : b;
    }
    else {
        return b;
    }
}

#endif
