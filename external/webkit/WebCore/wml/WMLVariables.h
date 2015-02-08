

#ifndef WMLVariables_h
#define WMLVariables_h

#if ENABLE(WML)
namespace WebCore {

    class Document;
    class String;

    enum WMLVariableEscapingMode {
        WMLVariableEscapingNone = 0,
        WMLVariableEscapingEscape,
        WMLVariableEscapingUnescape
    };

    bool isValidVariableName(const String&);
    bool containsVariableReference(const String&, bool& isValid);

    String substituteVariableReferences(const String& variableReference,
                                        Document*,
                                        WMLVariableEscapingMode mode = WMLVariableEscapingNone);

}

#endif
#endif
