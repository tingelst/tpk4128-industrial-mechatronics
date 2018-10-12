import socket
import pygame
import pygame.camera
from pygame.locals import *
import numpy as np
import pickle

DEVICE = '/dev/video0'
SIZE = (640, 480)

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port


def camstream():
    pygame.init()
    pygame.camera.init()
    camera = pygame.camera.Camera(DEVICE, SIZE, 'RGB')
    camera.start()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    print('Connected by', addr)

    while True:

        data = conn.recv(1024)
        print(data)
        if not data:
            break

        buf = camera.get_image()
        buf = pygame.surfarray.array2d(buf)
        buf = buf.tostring()
        sent = 0
        while buf:
            bytes_ = conn.send(buf)
            sent += bytes_
            buf = buf[bytes_:]
            print(sent)

    conn.close()

    camera.stop()
    pygame.quit()
    return


if __name__ == '__main__':
    camstream()
