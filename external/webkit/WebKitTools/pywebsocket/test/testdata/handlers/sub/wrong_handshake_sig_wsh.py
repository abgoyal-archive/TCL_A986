"""Wrong web_socket_do_extra_handshake signature.
"""


def no_web_socket_do_extra_handshake(request):
    pass


def web_socket_transfer_data(request):
    request.connection.write(
            'sub/wrong_handshake_sig_wsh.py is called for %s, %s' %
             (request.ws_resource, request.ws_protocol))


# vi:sts=4 sw=4 et
