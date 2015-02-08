
#include "unicode/utypes.h"

#if !UCONFIG_NO_NORMALIZATION

#include "unicode/unorm.h"
#include "unicode/ustring.h"
#include "cmemory.h"
#include "normalizer2impl.h"
#include "ucase.h"
#include "uprops.h"
#include "ustr_imp.h"

U_NAMESPACE_USE

#define LENGTHOF(array) (int32_t)(sizeof(array)/sizeof((array)[0]))

/* compare canonically equivalent ------------------------------------------- */


/* stack element for previous-level source/decomposition pointers */
struct CmpEquivLevel {
    const UChar *start, *s, *limit;
};
typedef struct CmpEquivLevel CmpEquivLevel;

#define _COMPARE_EQUIV 0x80000

/* internal function */
static int32_t
unorm_cmpEquivFold(const UChar *s1, int32_t length1,
                   const UChar *s2, int32_t length2,
                   uint32_t options,
                   UErrorCode *pErrorCode) {
    const Normalizer2Impl *nfcImpl;
    const UCaseProps *csp;

    /* current-level start/limit - s1/s2 as current */
    const UChar *start1, *start2, *limit1, *limit2;

    /* decomposition and case folding variables */
    const UChar *p;
    int32_t length;

    /* stacks of previous-level start/current/limit */
    CmpEquivLevel stack1[2], stack2[2];

    /* buffers for algorithmic decompositions */
    UChar decomp1[4], decomp2[4];

    /* case folding buffers, only use current-level start/limit */
    UChar fold1[UCASE_MAX_STRING_LENGTH+1], fold2[UCASE_MAX_STRING_LENGTH+1];

    /* track which is the current level per string */
    int32_t level1, level2;

    /* current code units, and code points for lookups */
    UChar32 c1, c2, cp1, cp2;

    /* no argument error checking because this itself is not an API */

    /*
     * assume that at least one of the options _COMPARE_EQUIV and U_COMPARE_IGNORE_CASE is set
     * otherwise this function must behave exactly as uprv_strCompare()
     * not checking for that here makes testing this function easier
     */

    /* normalization/properties data loaded? */
    if((options&_COMPARE_EQUIV)!=0) {
        nfcImpl=Normalizer2Factory::getNFCImpl(*pErrorCode);
    } else {
        nfcImpl=NULL;
    }
    if((options&U_COMPARE_IGNORE_CASE)!=0) {
        csp=ucase_getSingleton(pErrorCode);
    } else {
        csp=NULL;
    }
    if(U_FAILURE(*pErrorCode)) {
        return 0;
    }

    /* initialize */
    start1=s1;
    if(length1==-1) {
        limit1=NULL;
    } else {
        limit1=s1+length1;
    }

    start2=s2;
    if(length2==-1) {
        limit2=NULL;
    } else {
        limit2=s2+length2;
    }

    level1=level2=0;
    c1=c2=-1;

    /* comparison loop */
    for(;;) {
        /*
         * here a code unit value of -1 means "get another code unit"
         * below it will mean "this source is finished"
         */

        if(c1<0) {
            /* get next code unit from string 1, post-increment */
            for(;;) {
                if(s1==limit1 || ((c1=*s1)==0 && (limit1==NULL || (options&_STRNCMP_STYLE)))) {
                    if(level1==0) {
                        c1=-1;
                        break;
                    }
                } else {
                    ++s1;
                    break;
                }

                /* reached end of level buffer, pop one level */
                do {
                    --level1;
                    start1=stack1[level1].start;
                } while(start1==NULL);
                s1=stack1[level1].s;
                limit1=stack1[level1].limit;
            }
        }

        if(c2<0) {
            /* get next code unit from string 2, post-increment */
            for(;;) {
                if(s2==limit2 || ((c2=*s2)==0 && (limit2==NULL || (options&_STRNCMP_STYLE)))) {
                    if(level2==0) {
                        c2=-1;
                        break;
                    }
                } else {
                    ++s2;
                    break;
                }

                /* reached end of level buffer, pop one level */
                do {
                    --level2;
                    start2=stack2[level2].start;
                } while(start2==NULL);
                s2=stack2[level2].s;
                limit2=stack2[level2].limit;
            }
        }

        /*
         * compare c1 and c2
         * either variable c1, c2 is -1 only if the corresponding string is finished
         */
        if(c1==c2) {
            if(c1<0) {
                return 0;   /* c1==c2==-1 indicating end of strings */
            }
            c1=c2=-1;       /* make us fetch new code units */
            continue;
        } else if(c1<0) {
            return -1;      /* string 1 ends before string 2 */
        } else if(c2<0) {
            return 1;       /* string 2 ends before string 1 */
        }
        /* c1!=c2 && c1>=0 && c2>=0 */

        /* get complete code points for c1, c2 for lookups if either is a surrogate */
        cp1=c1;
        if(U_IS_SURROGATE(c1)) {
            UChar c;

            if(U_IS_SURROGATE_LEAD(c1)) {
                if(s1!=limit1 && U16_IS_TRAIL(c=*s1)) {
                    /* advance ++s1; only below if cp1 decomposes/case-folds */
                    cp1=U16_GET_SUPPLEMENTARY(c1, c);
                }
            } else /* isTrail(c1) */ {
                if(start1<=(s1-2) && U16_IS_LEAD(c=*(s1-2))) {
                    cp1=U16_GET_SUPPLEMENTARY(c, c1);
                }
            }
        }

        cp2=c2;
        if(U_IS_SURROGATE(c2)) {
            UChar c;

            if(U_IS_SURROGATE_LEAD(c2)) {
                if(s2!=limit2 && U16_IS_TRAIL(c=*s2)) {
                    /* advance ++s2; only below if cp2 decomposes/case-folds */
                    cp2=U16_GET_SUPPLEMENTARY(c2, c);
                }
            } else /* isTrail(c2) */ {
                if(start2<=(s2-2) && U16_IS_LEAD(c=*(s2-2))) {
                    cp2=U16_GET_SUPPLEMENTARY(c, c2);
                }
            }
        }

        /*
         * go down one level for each string
         * continue with the main loop as soon as there is a real change
         */

        if( level1==0 && (options&U_COMPARE_IGNORE_CASE) &&
            (length=ucase_toFullFolding(csp, (UChar32)cp1, &p, options))>=0
        ) {
            /* cp1 case-folds to the code point "length" or to p[length] */
            if(U_IS_SURROGATE(c1)) {
                if(U_IS_SURROGATE_LEAD(c1)) {
                    /* advance beyond source surrogate pair if it case-folds */
                    ++s1;
                } else /* isTrail(c1) */ {
                    /*
                     * we got a supplementary code point when hitting its trail surrogate,
                     * therefore the lead surrogate must have been the same as in the other string;
                     * compare this decomposition with the lead surrogate in the other string
                     * remember that this simulates bulk text replacement:
                     * the decomposition would replace the entire code point
                     */
                    --s2;
                    c2=*(s2-1);
                }
            }

            /* push current level pointers */
            stack1[0].start=start1;
            stack1[0].s=s1;
            stack1[0].limit=limit1;
            ++level1;

            /* copy the folding result to fold1[] */
            if(length<=UCASE_MAX_STRING_LENGTH) {
                u_memcpy(fold1, p, length);
            } else {
                int32_t i=0;
                U16_APPEND_UNSAFE(fold1, i, length);
                length=i;
            }

            /* set next level pointers to case folding */
            start1=s1=fold1;
            limit1=fold1+length;

            /* get ready to read from decomposition, continue with loop */
            c1=-1;
            continue;
        }

        if( level2==0 && (options&U_COMPARE_IGNORE_CASE) &&
            (length=ucase_toFullFolding(csp, (UChar32)cp2, &p, options))>=0
        ) {
            /* cp2 case-folds to the code point "length" or to p[length] */
            if(U_IS_SURROGATE(c2)) {
                if(U_IS_SURROGATE_LEAD(c2)) {
                    /* advance beyond source surrogate pair if it case-folds */
                    ++s2;
                } else /* isTrail(c2) */ {
                    /*
                     * we got a supplementary code point when hitting its trail surrogate,
                     * therefore the lead surrogate must have been the same as in the other string;
                     * compare this decomposition with the lead surrogate in the other string
                     * remember that this simulates bulk text replacement:
                     * the decomposition would replace the entire code point
                     */
                    --s1;
                    c1=*(s1-1);
                }
            }

            /* push current level pointers */
            stack2[0].start=start2;
            stack2[0].s=s2;
            stack2[0].limit=limit2;
            ++level2;

            /* copy the folding result to fold2[] */
            if(length<=UCASE_MAX_STRING_LENGTH) {
                u_memcpy(fold2, p, length);
            } else {
                int32_t i=0;
                U16_APPEND_UNSAFE(fold2, i, length);
                length=i;
            }

            /* set next level pointers to case folding */
            start2=s2=fold2;
            limit2=fold2+length;

            /* get ready to read from decomposition, continue with loop */
            c2=-1;
            continue;
        }

        if( level1<2 && (options&_COMPARE_EQUIV) &&
            0!=(p=nfcImpl->getDecomposition((UChar32)cp1, decomp1, length))
        ) {
            /* cp1 decomposes into p[length] */
            if(U_IS_SURROGATE(c1)) {
                if(U_IS_SURROGATE_LEAD(c1)) {
                    /* advance beyond source surrogate pair if it decomposes */
                    ++s1;
                } else /* isTrail(c1) */ {
                    /*
                     * we got a supplementary code point when hitting its trail surrogate,
                     * therefore the lead surrogate must have been the same as in the other string;
                     * compare this decomposition with the lead surrogate in the other string
                     * remember that this simulates bulk text replacement:
                     * the decomposition would replace the entire code point
                     */
                    --s2;
                    c2=*(s2-1);
                }
            }

            /* push current level pointers */
            stack1[level1].start=start1;
            stack1[level1].s=s1;
            stack1[level1].limit=limit1;
            ++level1;

            /* set empty intermediate level if skipped */
            if(level1<2) {
                stack1[level1++].start=NULL;
            }

            /* set next level pointers to decomposition */
            start1=s1=p;
            limit1=p+length;

            /* get ready to read from decomposition, continue with loop */
            c1=-1;
            continue;
        }

        if( level2<2 && (options&_COMPARE_EQUIV) &&
            0!=(p=nfcImpl->getDecomposition((UChar32)cp2, decomp2, length))
        ) {
            /* cp2 decomposes into p[length] */
            if(U_IS_SURROGATE(c2)) {
                if(U_IS_SURROGATE_LEAD(c2)) {
                    /* advance beyond source surrogate pair if it decomposes */
                    ++s2;
                } else /* isTrail(c2) */ {
                    /*
                     * we got a supplementary code point when hitting its trail surrogate,
                     * therefore the lead surrogate must have been the same as in the other string;
                     * compare this decomposition with the lead surrogate in the other string
                     * remember that this simulates bulk text replacement:
                     * the decomposition would replace the entire code point
                     */
                    --s1;
                    c1=*(s1-1);
                }
            }

            /* push current level pointers */
            stack2[level2].start=start2;
            stack2[level2].s=s2;
            stack2[level2].limit=limit2;
            ++level2;

            /* set empty intermediate level if skipped */
            if(level2<2) {
                stack2[level2++].start=NULL;
            }

            /* set next level pointers to decomposition */
            start2=s2=p;
            limit2=p+length;

            /* get ready to read from decomposition, continue with loop */
            c2=-1;
            continue;
        }

        /*
         * no decomposition/case folding, max level for both sides:
         * return difference result
         *
         * code point order comparison must not just return cp1-cp2
         * because when single surrogates are present then the surrogate pairs
         * that formed cp1 and cp2 may be from different string indexes
         *
         * example: { d800 d800 dc01 } vs. { d800 dc00 }, compare at second code units
         * c1=d800 cp1=10001 c2=dc00 cp2=10000
         * cp1-cp2>0 but c1-c2<0 and in fact in UTF-32 it is { d800 10001 } < { 10000 }
         *
         * therefore, use same fix-up as in ustring.c/uprv_strCompare()
         * except: uprv_strCompare() fetches c=*s while this functions fetches c=*s++
         * so we have slightly different pointer/start/limit comparisons here
         */

        if(c1>=0xd800 && c2>=0xd800 && (options&U_COMPARE_CODE_POINT_ORDER)) {
            /* subtract 0x2800 from BMP code points to make them smaller than supplementary ones */
            if(
                (c1<=0xdbff && s1!=limit1 && U16_IS_TRAIL(*s1)) ||
                (U16_IS_TRAIL(c1) && start1!=(s1-1) && U16_IS_LEAD(*(s1-2)))
            ) {
                /* part of a surrogate pair, leave >=d800 */
            } else {
                /* BMP code point - may be surrogate code point - make <d800 */
                c1-=0x2800;
            }

            if(
                (c2<=0xdbff && s2!=limit2 && U16_IS_TRAIL(*s2)) ||
                (U16_IS_TRAIL(c2) && start2!=(s2-1) && U16_IS_LEAD(*(s2-2)))
            ) {
                /* part of a surrogate pair, leave >=d800 */
            } else {
                /* BMP code point - may be surrogate code point - make <d800 */
                c2-=0x2800;
            }
        }

        return c1-c2;
    }
}

U_CAPI int32_t U_EXPORT2
unorm_compare(const UChar *s1, int32_t length1,
              const UChar *s2, int32_t length2,
              uint32_t options,
              UErrorCode *pErrorCode) {
    /* argument checking */
    if(U_FAILURE(*pErrorCode)) {
        return 0;
    }
    if(s1==0 || length1<-1 || s2==0 || length2<-1) {
        *pErrorCode=U_ILLEGAL_ARGUMENT_ERROR;
        return 0;
    }

    UnicodeString fcd1, fcd2;
    int32_t normOptions=(int32_t)(options>>UNORM_COMPARE_NORM_OPTIONS_SHIFT);
    options|=_COMPARE_EQUIV;

    /*
     * UAX #21 Case Mappings, as fixed for Unicode version 4
     * (see Jitterbug 2021), defines a canonical caseless match as
     *
     * A string X is a canonical caseless match
     * for a string Y if and only if
     * NFD(toCasefold(NFD(X))) = NFD(toCasefold(NFD(Y)))
     *
     * For better performance, we check for FCD (or let the caller tell us that
     * both strings are in FCD) for the inner normalization.
     * BasicNormalizerTest::FindFoldFCDExceptions() makes sure that
     * case-folding preserves the FCD-ness of a string.
     * The outer normalization is then only performed by unorm_cmpEquivFold()
     * when there is a difference.
     *
     * Exception: When using the Turkic case-folding option, we do perform
     * full NFD first. This is because in the Turkic case precomposed characters
     * with 0049 capital I or 0069 small i fold differently whether they
     * are first decomposed or not, so an FCD check - a check only for
     * canonical order - is not sufficient.
     */
    if(!(options&UNORM_INPUT_IS_FCD) || (options&U_FOLD_CASE_EXCLUDE_SPECIAL_I)) {
        const Normalizer2 *n2;
        if(options&U_FOLD_CASE_EXCLUDE_SPECIAL_I) {
            n2=Normalizer2Factory::getNFDInstance(*pErrorCode);
        } else {
            n2=Normalizer2Factory::getFCDInstance(*pErrorCode);
        }
        if (U_FAILURE(*pErrorCode)) {
            return 0;
        }

        // check if s1 and/or s2 fulfill the FCD conditions
        const UnicodeSet *uni32;
        if(normOptions&UNORM_UNICODE_3_2) {
            uni32=uniset_getUnicode32Instance(*pErrorCode);
        } else {
            uni32=NULL;  // unused
        }
        FilteredNormalizer2 fn2(*n2, *uni32);
        if(normOptions&UNORM_UNICODE_3_2) {
            n2=&fn2;
        }

        UnicodeString str1(length1<0, s1, length1);
        UnicodeString str2(length2<0, s2, length2);
        int32_t spanQCYes1=n2->spanQuickCheckYes(str1, *pErrorCode);
        int32_t spanQCYes2=n2->spanQuickCheckYes(str2, *pErrorCode);
        if(U_FAILURE(*pErrorCode)) {
            return 0;
        }

        /*
         * ICU 2.4 had a further optimization:
         * If both strings were not in FCD, then they were both NFD'ed,
         * and the _COMPARE_EQUIV option was turned off.
         * It is not entirely clear that this is valid with the current
         * definition of the canonical caseless match.
         * Therefore, ICU 2.6 removes that optimization.
         */

        if(spanQCYes1<str1.length()) {
            UnicodeString unnormalized=str1.tempSubString(spanQCYes1);
            fcd1.setTo(FALSE, str1.getBuffer(), spanQCYes1);
            n2->normalizeSecondAndAppend(fcd1, unnormalized, *pErrorCode);
            s1=fcd1.getBuffer();
            length1=fcd1.length();
        }
        if(spanQCYes2<str2.length()) {
            UnicodeString unnormalized=str2.tempSubString(spanQCYes2);
            fcd2.setTo(FALSE, str2.getBuffer(), spanQCYes2);
            n2->normalizeSecondAndAppend(fcd2, unnormalized, *pErrorCode);
            s2=fcd2.getBuffer();
            length2=fcd2.length();
        }
    }

    if(U_SUCCESS(*pErrorCode)) {
        return unorm_cmpEquivFold(s1, length1, s2, length2, options, pErrorCode);
    } else {
        return 0;
    }
}

#endif /* #if !UCONFIG_NO_NORMALIZATION */
