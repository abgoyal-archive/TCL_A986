

#include "config.h"
#include "Parser.h"

#include "Debugger.h"
#include "Lexer.h"
#include <wtf/HashSet.h>
#include <wtf/Vector.h>
#include <memory>

using std::auto_ptr;

#ifndef yyparse
extern int jscyyparse(void*);
#endif

#ifdef ANDROID_INSTRUMENT
#include "TimeCounter.h"
#endif

namespace JSC {

void Parser::parse(JSGlobalData* globalData, int* errLine, UString* errMsg)
{
#ifdef ANDROID_INSTRUMENT
    android::TimeCounter::start(android::TimeCounter::JavaScriptParseTimeCounter);
#endif
    m_sourceElements = 0;

    int defaultErrLine;
    UString defaultErrMsg;

    if (!errLine)
        errLine = &defaultErrLine;
    if (!errMsg)
        errMsg = &defaultErrMsg;

    *errLine = -1;
    *errMsg = UString();

    Lexer& lexer = *globalData->lexer;
    lexer.setCode(*m_source, m_arena);

    int parseError = jscyyparse(globalData);
    bool lexError = lexer.sawError();
    int lineNumber = lexer.lineNumber();
    lexer.clear();

    if (parseError || lexError) {
        *errLine = lineNumber;
        *errMsg = "Parse error";
        m_sourceElements = 0;
    }
#ifdef ANDROID_INSTRUMENT
    android::TimeCounter::record(android::TimeCounter::JavaScriptParseTimeCounter, __FUNCTION__);
#endif
}

void Parser::didFinishParsing(SourceElements* sourceElements, ParserArenaData<DeclarationStacks::VarStack>* varStack, 
                              ParserArenaData<DeclarationStacks::FunctionStack>* funcStack, CodeFeatures features, int lastLine, int numConstants)
{
    m_sourceElements = sourceElements;
    m_varDeclarations = varStack;
    m_funcDeclarations = funcStack;
    m_features = features;
    m_lastLine = lastLine;
    m_numConstants = numConstants;
}

} // namespace JSC
