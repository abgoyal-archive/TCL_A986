"""Wrong web_socket_transfer_data() signature.
"""


def web_socket_do_extra_handshake(request):
    pass


def no_web_socket_transfer_data(request):
    request.connection.write(
            'sub/wrong_transfer_sig_wsh.py is called for %s, %s' %
            (request.ws_resource, request.ws_protocol))


# vi:sts=4 sw=4 et
