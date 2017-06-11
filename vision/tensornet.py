import cv2
import numpy as np

class TensorNet:
  def __init__(self,netname):
    self.name = netname

  def process(self,image):   #272x512 RGB 32F input
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    g8uc = (gray*256.0).astype(np.uint8)
    img = g8uc[::16,::16] #34x64 Grayscale 8UC output
    return img

  def subset(self,image):
    return np.ravel(image[14:20,8:24])

  def fini(self):
    return True

#import tensorflow as tf
#hello = tf.constant("Tensorflow is working")
#sess = tf.Session()
#print(sess.run(hello))

