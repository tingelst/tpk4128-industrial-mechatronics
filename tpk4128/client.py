import socket

class SocketClient(object):
    
    def __init__(self, host, port):
        self._host = host
        self._port = port

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._host, self._port))

    def sendall(self, data):
        self._socket.sendall(data)

    def recv(self, size):
        buf = b''
        received = 0
        while received < size:
            data = self._socket.recv(4096)
            buf += data
            if not data:
                break
            received += len(data)
        return received, buf

    def __del__(self):
        self._socket.close()

