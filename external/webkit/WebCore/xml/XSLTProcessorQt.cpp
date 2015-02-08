

#include "config.h"

#if ENABLE(XSLT)

#include "XSLTProcessor.h"

#include "Console.h"
#include "DOMWindow.h"
#include "Frame.h"
#include "TransformSource.h"
#include "loader.h"
#include "markup.h"
#include <wtf/Assertions.h>
#include <wtf/Platform.h>
#include <wtf/Vector.h>

#include <qabstractmessagehandler.h>
#include <qabstracturiresolver.h>
#include <qbuffer.h>
#include <qsourcelocation.h>
#include <qxmlquery.h>

namespace WebCore {

class XSLTMessageHandler : public QAbstractMessageHandler {

public:
    XSLTMessageHandler(Document* document = 0);
    virtual void handleMessage(QtMsgType type, const QString& description,
                               const QUrl& identifier, const QSourceLocation& sourceLocation);

private:
    Document* m_document;
};

XSLTMessageHandler::XSLTMessageHandler(Document* document)
    : QAbstractMessageHandler()
    , m_document(document)
{
}

void XSLTMessageHandler::handleMessage(QtMsgType type, const QString& description,
                                       const QUrl&, const QSourceLocation& sourceLocation)
{
    if (!m_document->frame())
        return;

    MessageLevel level;
    switch (type) {
    case QtDebugMsg:
        level = TipMessageLevel;
        break;
    case QtWarningMsg:
        level = WarningMessageLevel;
        break;
    case QtCriticalMsg:
    case QtFatalMsg:
        level = ErrorMessageLevel;
        break;
    default:
        level = LogMessageLevel;
        break;
    }

    Console* console = m_document->frame()->domWindow()->console();
    console->addMessage(XMLMessageSource, LogMessageType, level, description,
                        sourceLocation.line(), sourceLocation.uri().toString());
}

class XSLTUriResolver : public QAbstractUriResolver {

public:
    XSLTUriResolver(Document* document);
    virtual QUrl resolve(const QUrl& relative, const QUrl& baseURI) const;

private:
    Document* m_document;
};

XSLTUriResolver::XSLTUriResolver(Document* document)
    : QAbstractUriResolver()
    , m_document(document)
{
}

QUrl XSLTUriResolver::resolve(const QUrl& relative, const QUrl& baseURI) const
{
    QUrl url = baseURI.resolved(relative);

    if (!m_document->frame() || !m_document->securityOrigin()->canRequest(url))
        return QUrl();
    return url;
}

bool XSLTProcessor::transformToString(Node* sourceNode, String&, String& resultString, String&)
{
    bool success = false;

    RefPtr<XSLStyleSheet> stylesheet = m_stylesheet;
    if (!stylesheet && m_stylesheetRootNode) {
        Node* node = m_stylesheetRootNode.get();
        stylesheet = XSLStyleSheet::create(node->parent() ? node->parent() : node,
            node->document()->url().string(),
            node->document()->url()); // FIXME: Should we use baseURL here?
        stylesheet->parseString(createMarkup(node));
    }

    if (!stylesheet || stylesheet->sheetString().isEmpty())
        return success;

    RefPtr<Document> ownerDocument = sourceNode->document();
    bool sourceIsDocument = (sourceNode == ownerDocument.get());

    QXmlQuery query(QXmlQuery::XSLT20);

    XSLTMessageHandler messageHandler(ownerDocument.get());
    XSLTUriResolver uriResolver(ownerDocument.get());
    query.setMessageHandler(&messageHandler);

    XSLTProcessor::ParameterMap::iterator end = m_parameters.end();
    for (XSLTProcessor::ParameterMap::iterator it = m_parameters.begin(); it != end; ++it)
        query.bindVariable(QString(it->first), QXmlItem(QVariant(it->second)));

    QString source;
    if (sourceIsDocument && ownerDocument->transformSource())
        source = ownerDocument->transformSource()->platformSource();
    if (!sourceIsDocument || source.isEmpty())
        source = createMarkup(sourceNode);

    QBuffer inputBuffer;
    QBuffer styleSheetBuffer;
    QBuffer outputBuffer;

    inputBuffer.setData(source.toUtf8());
    styleSheetBuffer.setData(QString(stylesheet->sheetString()).toUtf8());

    inputBuffer.open(QIODevice::ReadOnly);
    styleSheetBuffer.open(QIODevice::ReadOnly);
    outputBuffer.open(QIODevice::ReadWrite);

    query.setFocus(&inputBuffer);
    query.setQuery(&styleSheetBuffer, QUrl(stylesheet->href()));

    query.setUriResolver(&uriResolver);

    success = query.evaluateTo(&outputBuffer);
    outputBuffer.reset();
    resultString = QString::fromUtf8(outputBuffer.readAll()).trimmed();

    if (m_stylesheet) {
        m_stylesheet->clearDocuments();
        m_stylesheet = 0;
    }

    return success;
}

} // namespace WebCore

#endif // ENABLE(XSLT)
