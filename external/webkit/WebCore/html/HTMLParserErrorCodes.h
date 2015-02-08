

#ifndef HTMLParserErrorCodes_h
#define HTMLParserErrorCodes_h

namespace WebCore {

enum HTMLParserErrorCode { 
    MisplacedTablePartError,
    MisplacedHeadError,
    MisplacedHeadContentError,
    RedundantHTMLBodyError,
    MisplacedAreaError,
    IgnoredContentError,
    MisplacedFramesetContentError,
    MisplacedContentRetryError,
    MisplacedCaptionContentError,
    MisplacedTableError,
    StrayTableContentError,
    TablePartRequiredError,
    MalformedBRError,
    IncorrectXMLSelfCloseError,
    StrayParagraphCloseError,
    StrayCloseTagError,
    ResidualStyleError,
    FormInsideTablePartError,
    IncorrectXMLCloseScriptWarning
};

const char* htmlParserErrorMessageTemplate(HTMLParserErrorCode);
const char* htmlParserDocumentWriteMessage();

bool isWarning(HTMLParserErrorCode);

}

#endif
