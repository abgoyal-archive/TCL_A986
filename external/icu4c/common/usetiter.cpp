#include "unicode/usetiter.h"
#include "unicode/uniset.h"
#include "unicode/unistr.h"
#include "uvector.h"

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(UnicodeSetIterator)

UnicodeSetIterator::UnicodeSetIterator(const UnicodeSet& uSet) {
    cpString  = NULL;
    reset(uSet);
}

UnicodeSetIterator::UnicodeSetIterator() {
    this->set = NULL;
    cpString  = NULL;
    reset();
}

UnicodeSetIterator::~UnicodeSetIterator() {
    delete cpString;
}

UBool UnicodeSetIterator::next() {
    if (nextElement <= endElement) {
        codepoint = codepointEnd = nextElement++;
        string = NULL;
        return TRUE;
    }
    if (range < endRange) {
        loadRange(++range);
        codepoint = codepointEnd = nextElement++;
        string = NULL;
        return TRUE;
    }

    if (nextString >= stringCount) return FALSE;
    codepoint = (UChar32)IS_STRING; // signal that value is actually a string
    string = (const UnicodeString*) set->strings->elementAt(nextString++);
    return TRUE;
}

UBool UnicodeSetIterator::nextRange() {
    string = NULL;
    if (nextElement <= endElement) {
        codepointEnd = endElement;
        codepoint = nextElement;
        nextElement = endElement+1;
        return TRUE;
    }
    if (range < endRange) {
        loadRange(++range);
        codepointEnd = endElement;
        codepoint = nextElement;
        nextElement = endElement+1;
        return TRUE;
    }

    if (nextString >= stringCount) return FALSE;
    codepoint = (UChar32)IS_STRING; // signal that value is actually a string
    string = (const UnicodeString*) set->strings->elementAt(nextString++);
    return TRUE;
}

void UnicodeSetIterator::reset(const UnicodeSet& uSet) {
    this->set = &uSet;
    reset();
}

void UnicodeSetIterator::reset() {
    if (set == NULL) {
        // Set up indices to empty iteration
        endRange = -1;
        stringCount = 0;
    } else {
        endRange = set->getRangeCount() - 1;
        stringCount = set->strings->size();
    }
    range = 0;
    endElement = -1;
    nextElement = 0;            
    if (endRange >= 0) {
        loadRange(range);
    }
    nextString = 0;
    string = NULL;
}

void UnicodeSetIterator::loadRange(int32_t iRange) {
    nextElement = set->getRangeStart(iRange);
    endElement = set->getRangeEnd(iRange);
}


const UnicodeString& UnicodeSetIterator::getString()  {
    if (string==NULL && codepoint!=(UChar32)IS_STRING) {
       if (cpString == NULL) {
          cpString = new UnicodeString();
       }
       if (cpString != NULL) {
          cpString->setTo((UChar32)codepoint);
       }
       string = cpString;
    }
    return *string;
}

U_NAMESPACE_END

//eof
