"""Exception in web_socket_transfer_data().
"""


def web_socket_do_extra_handshake(request):
    pass


def web_socket_transfer_data(request):
    raise Exception('Intentional Exception for %s, %s' %
                    (request.ws_resource, request.ws_protocol))


# vi:sts=4 sw=4 et
