

#ifndef LiteralParser_h
#define LiteralParser_h

#include "JSGlobalObjectFunctions.h"
#include "JSValue.h"
#include "UString.h"

namespace JSC {

    class LiteralParser {
    public:
        typedef enum { StrictJSON, NonStrictJSON } ParserMode;
        LiteralParser(ExecState* exec, const UString& s, ParserMode mode)
            : m_exec(exec)
            , m_lexer(s, mode)
            , m_mode(mode)
        {
        }
        
        JSValue tryLiteralParse()
        {
            m_lexer.next();
            JSValue result = parse(m_mode == StrictJSON ? StartParseExpression : StartParseStatement);
            if (m_lexer.currentToken().type != TokEnd)
                return JSValue();
            return result;
        }
    private:
        enum ParserState { StartParseObject, StartParseArray, StartParseExpression, 
                           StartParseStatement, StartParseStatementEndStatement, 
                           DoParseObjectStartExpression, DoParseObjectEndExpression,
                           DoParseArrayStartExpression, DoParseArrayEndExpression };
        enum TokenType { TokLBracket, TokRBracket, TokLBrace, TokRBrace, 
                         TokString, TokIdentifier, TokNumber, TokColon, 
                         TokLParen, TokRParen, TokComma, TokTrue, TokFalse,
                         TokNull, TokEnd, TokError };

        class Lexer {
        public:
            struct LiteralParserToken {
                TokenType type;
                const UChar* start;
                const UChar* end;
                UString stringToken;
                double numberToken;
            };
            Lexer(const UString& s, ParserMode mode)
                : m_string(s)
                , m_mode(mode)
                , m_ptr(s.data())
                , m_end(s.data() + s.size())
            {
            }
            
            TokenType next()
            {
                return lex(m_currentToken);
            }
            
            const LiteralParserToken& currentToken()
            {
                return m_currentToken;
            }
            
        private:
            TokenType lex(LiteralParserToken&);
            template <ParserMode mode> TokenType lexString(LiteralParserToken&);
            TokenType lexNumber(LiteralParserToken&);
            LiteralParserToken m_currentToken;
            UString m_string;
            ParserMode m_mode;
            const UChar* m_ptr;
            const UChar* m_end;
        };
        
        class StackGuard;
        JSValue parse(ParserState);

        ExecState* m_exec;
        LiteralParser::Lexer m_lexer;
        ParserMode m_mode;
    };
}

#endif
