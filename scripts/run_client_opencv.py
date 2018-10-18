from tpk4128.client import SocketClient
import time
import cv2
import numpy as np


def main():

    client = SocketClient('localhost', 50007)
    while True:
        client.sendall(b'Hello World!')

        # Tip: len(img.tostring())
        size, data = client.recv(INSERT THE CORRECT IMAGE SIZE HERE)
        if not data:
            break

        # Tip: img.dtype, img.shape
        img = np.frombuffer(data, INSERT THE CORRECT DATA TYPE HERE).reshape(INSERT THE CORRECT IMAGE SHAPE HERE)

        cv2.imshow('img', img)
        if cv2.waitKey(20) == 27:  # Esc: 27
            break


if __name__ == '__main__':
    main()
