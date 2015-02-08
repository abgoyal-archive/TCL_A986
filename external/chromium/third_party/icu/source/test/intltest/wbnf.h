
#ifndef _WBNF
#define _WBNF

#include "unicode/utypes.h"

const int DEFAULT_WEIGHT = 100;
const int PSEUDO_INFINIT = 200;

class LanguageGenerator_impl;

class LanguageGenerator{
    LanguageGenerator_impl * lang_gen;
public:
    enum PARSE_RESULT {OK, BNF_DEF_WRONG, INCOMPLETE, NO_TOP_NODE};
    LanguageGenerator();
    ~LanguageGenerator();
    PARSE_RESULT parseBNF(const char *const bnf_definition /*in*/, const char *const top_node/*in*/, UBool debug=FALSE);
    const char *next(); /* Return a null-terminated c-string. The buffer is owned by callee. */
};

void TestWbnf(void);

#endif /* _WBNF */
