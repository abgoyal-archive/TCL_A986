

#ifndef WMLErrorCodes_h
#define WMLErrorCodes_h

#if ENABLE(WML)
namespace WebCore {

    class Document;
    class String;

    enum WMLErrorCode {
        WMLErrorUnknown = 0,
        WMLErrorConflictingEventBinding,
        WMLErrorDeckNotAccessible,
        WMLErrorDuplicatedDoElement,
        WMLErrorForbiddenTaskInAnchorElement,
        WMLErrorInvalidColumnsNumberInTable,
        WMLErrorInvalidVariableName,
        WMLErrorInvalidVariableReference,
        WMLErrorInvalidVariableReferenceLocation,
        WMLErrorMultipleAccessElements,
        WMLErrorMultipleTemplateElements,
        WMLErrorMultipleTimerElements,
        WMLErrorNoCardInDocument
    };

    String errorMessageForErrorCode(WMLErrorCode);
    void reportWMLError(Document*, WMLErrorCode);
}

#endif
#endif
