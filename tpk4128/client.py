# Echo client program
import socket
from PIL import Image
import numpy as np
import struct
import matplotlib.pyplot as plt
import cv2
import pickle


def image(rgba):
    blue = np.bitwise_and(np.right_shift(rgba, 0), 0xff)
    green = np.bitwise_and(np.right_shift(rgba, 8), 0xff)
    red = np.bitwise_and(np.right_shift(rgba, 16), 0xff)
    alpha = np.bitwise_and(np.right_shift(rgba, 24), 0xff)
    im = np.stack((red, green, blue, alpha), axis=2) / 255.0
    return im


HOST = '10.42.0.1'    # The remote host
PORT = 50007          # The same port as used by the server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))

    k = 0
    while True:
        s.sendall(b'Hello, world')

        img = b''
        size = 0
        while size < 1228800:

            data = s.recv(4096)
            img += data

            if not data:
                break

            size += len(data)

        img = np.frombuffer(img, np.int32).reshape(640, 480).T
        img = image(img)

        cv2.imshow('img', img)
        if cv2.waitKey(30) == 27:  # Esc: 27
            break
