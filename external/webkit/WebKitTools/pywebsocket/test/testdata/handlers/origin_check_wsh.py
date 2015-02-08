def web_socket_do_extra_handshake(request):
    if request.ws_origin == 'http://example.com':
        return
    raise ValueError('Unacceptable origin: %r' % request.ws_origin)


def web_socket_transfer_data(request):
    request.connection.write('origin_check_wsh.py is called for %s, %s' %
                             (request.ws_resource, request.ws_protocol))


# vi:sts=4 sw=4 et
