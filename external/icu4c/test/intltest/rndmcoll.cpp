#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "rndmcoll.h"
#include "wbnf.h"


#if !UCONFIG_NO_COLLATION

//Raymond: Following comments are copied from Java implementation
//
// each rule can be:
//      "[" command "]"
//      "& [" position "]"
//      "&" before chars
//      relation "[variable top]"
//      relation (chars "|")? chars ("/" chars)?
// plus, a reset must come before a relation


static const char collationBNF[] =
    "$s = ' '? 50%;" 
    "$crlf = '\r\n';" 

    "$alternateOptions = non'-'ignorable | shifted;" 
    "$onoff = on | off;" 
    "$caseFirstOptions = off | upper | lower;" 
    "$strengthOptions = '1' | '2' | '3' | '4' | 'I';" 
    "$commandList = '['"
    " ( alternate ' ' $alternateOptions"
    " | backwards' 2'"
    " | normalization ' ' $onoff "
    " | caseLevel ' ' $onoff "
    " | hiraganaQ ' ' $onoff"
    " | caseFirst ' ' $caseFirstOptions"
    " | strength ' ' $strengthOptions"
    " ) ']';" 
    "$command = $commandList $crlf;" 

    "$ignorableTypes = (tertiary | secondary | primary) ' ' ignorable;" 
    "$allTypes = variable | regular | implicit | trailing | $ignorableTypes;" 
    "$positionList = '[' (first | last) ' ' $allTypes ']';"

    "$beforeList = '[before ' ('1' | '2' | '3') ']';"

    "$relationList = ("
    "   '<'"
    " | '<<'"
    " | ';'" 
    " | '<<<'"
    " | ','" 
    " | '='"
    ");"
    "$string = $chars{1,5}~@;" 
    "$chars = a|b|c|d|e|f|g|h|i|j|k|l|m|n|o|p|q|r|s|t|u|v|w|x|y|z|A|B|C|D|E|F|G|H|I|J|K|L|M|N|O|P|Q|R|S|T|U|V|W|X|Y|Z| '<'| '&'| '['| ']';"
    "$rel1 = '[variable top]' $s;" 
    "$p1 = ($string $s '|' $s)? 25%;" 
    "$p2 = ('/' $s $string $s)? 25%;" 
    "$rel2 = $p1 $string $s $p2;" 
    "$relation = $relationList $s ($rel1 | $rel2) $crlf;" 

    "$reset = '&' $s ($beforeList $s)? 10% ($positionList 1% | $string 10%) $crlf;" 
    "$mostRules = $command 1% | $reset 5% | $relation 25%;"
    "$root = $command{0,5} $reset $mostRules{1,20};";


void RandomCollatorTest::Test2(){
    // See ticket 5747 about reenabling this test.
    errln("TestWbnf is incorrectly implemented.\nThis test should be modeled to use the existing test frame work for naming tests.\n");
    TestWbnf();
}


void RandomCollatorTest::runIndexedTest( int32_t index, UBool exec, const char* &name, char* ){
    if (exec) logln("TestSuite RandomCollatorTest: ");
    switch (index) {
        TESTCASE(0, Test);
        TESTCASE(1, Test2);
        default: name = ""; break;
    }
}


void RandomCollatorTest::Test(){
    // See ticket 5747 about reenabling this test.
    errln("This test needs to be fixed.\n");

    LanguageGenerator test_rule;
    if (test_rule.parseBNF(collationBNF, "$root", TRUE) != LanguageGenerator::OK){
        errln("The test code itself is wrong.");
        return;
    };

    //TestColltorCompare coll_test;

    static const int CONSTRUCT_RANDOM_COUNT = 1000;
    int i;
    for (i=0; i < CONSTRUCT_RANDOM_COUNT; i++){
        const char * rule = test_rule.next();
        logln("\n-----------------------------------%d\n",i);
        logln(UnicodeString(rule, strlen(rule)));

        UnicodeString newRule(rule);    // potential bug
        UErrorCode status = U_ZERO_ERROR;
        logln(   "===========================================\n");
        fwrite(rule, strlen(rule),1,stdout);
        logln("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

        Collator * c = new RuleBasedCollator(newRule,status);

        if (U_FAILURE(status)) {
            errln( "Could not create Collator for the %d(th) generated rule.\n"
                   "Error Name: %s\n"
                   "The rule is ",
                   i, u_errorName(status));
            return;
        }

        delete c;
    }
}

#endif /* #if !UCONFIG_NO_COLLATION */

