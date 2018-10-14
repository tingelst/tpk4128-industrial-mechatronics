import socket

class SocketServer(object):

    def __init__(self, host, port):
        self._host = host
        self._port = port

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._host, self._port))
        self._socket.listen(1)

        self._conn, self._addr = self._socket.accept()
        print('Connected by: {}'.format(self._addr))

    def sendall(self, buf):
        self._conn.sendall(buf)

    def recv(self, size=1024):
        data = self._conn.recv(size)
        return data

    def __del__(self):
        self._conn.close()

        


