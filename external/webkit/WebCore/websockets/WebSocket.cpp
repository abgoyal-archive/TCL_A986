

#include "config.h"

#if ENABLE(WEB_SOCKETS)

#include "WebSocket.h"

#include "CString.h"
#include "DOMWindow.h"
#include "Event.h"
#include "EventException.h"
#include "EventListener.h"
#include "EventNames.h"
#include "Logging.h"
#include "MessageEvent.h"
#include "ScriptExecutionContext.h"
#include "StringBuilder.h"
#include "ThreadableWebSocketChannel.h"
#include "WebSocketChannel.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

static bool isValidProtocolString(const String& protocol)
{
    if (protocol.isNull())
        return true;
    if (protocol.isEmpty())
        return false;
    const UChar* characters = protocol.characters();
    for (size_t i = 0; i < protocol.length(); i++) {
        if (characters[i] < 0x20 || characters[i] > 0x7E)
            return false;
    }
    return true;
}

static String encodeProtocolString(const String& protocol)
{
    StringBuilder builder;
    for (size_t i = 0; i < protocol.length(); i++) {
        if (protocol[i] < 0x20 || protocol[i] > 0x7E)
            builder.append(String::format("\\u%04X", protocol[i]));
        else if (protocol[i] == 0x5c)
            builder.append("\\\\");
        else
            builder.append(protocol[i]);
    }
    return builder.toString();
}

#if USE(V8)

static bool webSocketsAvailable = false;

void WebSocket::setIsAvailable(bool available)
{
    webSocketsAvailable = available;
}

bool WebSocket::isAvailable()
{
    return webSocketsAvailable;
}

#endif

WebSocket::WebSocket(ScriptExecutionContext* context)
    : ActiveDOMObject(context, this)
    , m_state(CONNECTING)
    , m_bufferedAmountAfterClose(0)
{
}

WebSocket::~WebSocket()
{
    if (m_channel)
        m_channel->disconnect();
}

void WebSocket::connect(const KURL& url, ExceptionCode& ec)
{
    connect(url, String(), ec);
}

void WebSocket::connect(const KURL& url, const String& protocol, ExceptionCode& ec)
{
    LOG(Network, "WebSocket %p connect to %s protocol=%s", this, url.string().utf8().data(), protocol.utf8().data());
    m_url = url;
    m_protocol = protocol;

    if (!m_url.isValid()) {
        scriptExecutionContext()->addMessage(ConsoleDestination, JSMessageSource, LogMessageType, ErrorMessageLevel, "Invalid url for WebSocket " + url.string(), 0, scriptExecutionContext()->securityOrigin()->toString());
        m_state = CLOSED;
        ec = SYNTAX_ERR;
        return;
    }

    if (!m_url.protocolIs("ws") && !m_url.protocolIs("wss")) {
        scriptExecutionContext()->addMessage(ConsoleDestination, JSMessageSource, LogMessageType, ErrorMessageLevel, "Wrong url scheme for WebSocket " + url.string(), 0, scriptExecutionContext()->securityOrigin()->toString());
        m_state = CLOSED;
        ec = SYNTAX_ERR;
        return;
    }
    if (m_url.hasFragmentIdentifier()) {
        scriptExecutionContext()->addMessage(ConsoleDestination, JSMessageSource, LogMessageType, ErrorMessageLevel, "URL has fragment component " + url.string(), 0, scriptExecutionContext()->securityOrigin()->toString());
        m_state = CLOSED;
        ec = SYNTAX_ERR;
        return;
    }
    if (!isValidProtocolString(m_protocol)) {
        scriptExecutionContext()->addMessage(ConsoleDestination, JSMessageSource, LogMessageType, ErrorMessageLevel, "Wrong protocol for WebSocket '" + encodeProtocolString(m_protocol) + "'", 0, scriptExecutionContext()->securityOrigin()->toString());
        m_state = CLOSED;
        ec = SYNTAX_ERR;
        return;
    }
    if (!portAllowed(url)) {
        scriptExecutionContext()->addMessage(ConsoleDestination, JSMessageSource, LogMessageType, ErrorMessageLevel, String::format("WebSocket port %d blocked", url.port()), 0, scriptExecutionContext()->securityOrigin()->toString());
        m_state = CLOSED;
        ec = SECURITY_ERR;
        return;
    }

    m_channel = ThreadableWebSocketChannel::create(scriptExecutionContext(), this, m_url, m_protocol);
    m_channel->connect();
    ActiveDOMObject::setPendingActivity(this);
}

bool WebSocket::send(const String& message, ExceptionCode& ec)
{
    LOG(Network, "WebSocket %p send %s", this, message.utf8().data());
    if (m_state == CONNECTING) {
        ec = INVALID_STATE_ERR;
        return false;
    }
    // No exception is raised if the connection was once established but has subsequently been closed.
    if (m_state == CLOSED) {
        m_bufferedAmountAfterClose += message.utf8().length() + 2; // 2 for framing
        return false;
    }
    // FIXME: check message is valid utf8.
    ASSERT(m_channel);
    return m_channel->send(message);
}

void WebSocket::close()
{
    LOG(Network, "WebSocket %p close", this);
    if (m_state == CLOSED)
        return;
    m_state = CLOSED;
    m_bufferedAmountAfterClose = m_channel->bufferedAmount();
    m_channel->close();
}

const KURL& WebSocket::url() const
{
    return m_url;
}

WebSocket::State WebSocket::readyState() const
{
    return m_state;
}

unsigned long WebSocket::bufferedAmount() const
{
    if (m_state == OPEN)
        return m_channel->bufferedAmount();
    return m_bufferedAmountAfterClose;
}

ScriptExecutionContext* WebSocket::scriptExecutionContext() const
{
    return ActiveDOMObject::scriptExecutionContext();
}

void WebSocket::contextDestroyed()
{
    LOG(Network, "WebSocket %p scriptExecutionContext destroyed", this);
    ASSERT(!m_channel);
    ASSERT(m_state == CLOSED);
    ActiveDOMObject::contextDestroyed();
}

void WebSocket::stop()
{
    bool pending = hasPendingActivity();
    if (m_channel)
        m_channel->disconnect();
    m_channel = 0;
    m_state = CLOSED;
    ActiveDOMObject::stop();
    if (pending)
        ActiveDOMObject::unsetPendingActivity(this);
}

void WebSocket::didConnect()
{
    LOG(Network, "WebSocket %p didConnect", this);
    if (m_state != CONNECTING || !scriptExecutionContext()) {
        didClose(0);
        return;
    }
    m_state = OPEN;
    dispatchEvent(Event::create(eventNames().openEvent, false, false));
}

void WebSocket::didReceiveMessage(const String& msg)
{
    LOG(Network, "WebSocket %p didReceiveMessage %s", this, msg.utf8().data());
    if (m_state != OPEN || !scriptExecutionContext())
        return;
    RefPtr<MessageEvent> evt = MessageEvent::create();
    evt->initMessageEvent(eventNames().messageEvent, false, false, SerializedScriptValue::create(msg), "", "", 0, 0);
    dispatchEvent(evt);
}

void WebSocket::didClose(unsigned long unhandledBufferedAmount)
{
    LOG(Network, "WebSocket %p didClose", this);
    m_state = CLOSED;
    m_bufferedAmountAfterClose += unhandledBufferedAmount;
    dispatchEvent(Event::create(eventNames().closeEvent, false, false));
    m_channel = 0;
    if (hasPendingActivity())
        ActiveDOMObject::unsetPendingActivity(this);
}

EventTargetData* WebSocket::eventTargetData()
{
    return &m_eventTargetData;
}

EventTargetData* WebSocket::ensureEventTargetData()
{
    return &m_eventTargetData;
}

}  // namespace WebCore

#endif
