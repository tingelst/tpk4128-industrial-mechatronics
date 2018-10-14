import pygame
import pygame.camera
from pygame.locals import *

DEVICE = '/dev/video0'
SIZE = (640, 480)
FILENAME = 'capture.png'

class Camera(object):
    
    def __init__(self, size=(640,480), device='/dev/video0'):
        self._size = size
        self._device = device

        pygame.init()
        pygame.camera.init()

        self._camera = pygame.camera.Camera(self._device, self._size)
        self._camera.start()

    def capture(self, raw=False):
        if raw: 
            img = self._camera.get_raw()
        else:
            img = self._camera.get_image()
            img = pygame.surfarray.array2d(img)
        return img

    def __del__(self):
        self._camera.stop()
        pygame.quit()

        
        
