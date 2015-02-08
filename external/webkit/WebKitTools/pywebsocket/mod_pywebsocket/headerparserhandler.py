"""PythonHeaderParserHandler for mod_pywebsocket.

Apache HTTP Server and mod_python must be configured such that this
function is called to handle Web Socket request.
"""


from mod_python import apache

import dispatch
import handshake
import util


# PythonOption to specify the handler root directory.
_PYOPT_HANDLER_ROOT = 'mod_pywebsocket.handler_root'

# PythonOption to specify the handler scan directory.
# This must be a directory under the root directory.
# The default is the root directory.
_PYOPT_HANDLER_SCAN = 'mod_pywebsocket.handler_scan'


def _create_dispatcher():
    _HANDLER_ROOT = apache.main_server.get_options().get(
            _PYOPT_HANDLER_ROOT, None)
    if not _HANDLER_ROOT:
        raise Exception('PythonOption %s is not defined' % _PYOPT_HANDLER_ROOT,
                        apache.APLOG_ERR)
    _HANDLER_SCAN = apache.main_server.get_options().get(
            _PYOPT_HANDLER_SCAN, _HANDLER_ROOT)
    dispatcher = dispatch.Dispatcher(_HANDLER_ROOT, _HANDLER_SCAN)
    for warning in dispatcher.source_warnings():
        apache.log_error('mod_pywebsocket: %s' % warning, apache.APLOG_WARNING)
    return dispatcher


# Initialize
_dispatcher = _create_dispatcher()


def headerparserhandler(request):
    """Handle request.

    Args:
        request: mod_python request.

    This function is named headerparserhandler because it is the default name
    for a PythonHeaderParserHandler.
    """

    try:
        handshaker = handshake.Handshaker(request, _dispatcher)
        handshaker.do_handshake()
        request.log_error('mod_pywebsocket: resource: %r' % request.ws_resource,
                          apache.APLOG_DEBUG)
        _dispatcher.transfer_data(request)
    except handshake.HandshakeError, e:
        # Handshake for ws/wss failed.
        # But the request can be valid http/https request.
        request.log_error('mod_pywebsocket: %s' % e, apache.APLOG_INFO)
        return apache.DECLINED
    except dispatch.DispatchError, e:
        request.log_error('mod_pywebsocket: %s' % e, apache.APLOG_WARNING)
        return apache.DECLINED
    return apache.DONE  # Return DONE such that no other handlers are invoked.


# vi:sts=4 sw=4 et
