from tpk4128.client import SocketClient
import time
import cv2
import numpy as np


def image(rgba):
    blue = np.bitwise_and(np.right_shift(rgba, 0), 0xff)
    green = np.bitwise_and(np.right_shift(rgba, 8), 0xff)
    red = np.bitwise_and(np.right_shift(rgba, 16), 0xff)
    alpha = np.bitwise_and(np.right_shift(rgba, 24), 0xff)
    im = np.stack((blue, green, red, alpha), axis=2) / 255.0
    return im


def main():

    client = SocketClient('localhost', 50007)
    while True:
        client.sendall(b'Hello World!')

        size, data = client.recv(1228800)
        print(size)
        if not data:
            break

        img = np.frombuffer(data, np.int32).reshape(640, 480).T
        img = image(img)

        cv2.imshow('img', img)
        if cv2.waitKey(20) == 27:  # Esc: 27
            break


if __name__ == '__main__':
    main()
