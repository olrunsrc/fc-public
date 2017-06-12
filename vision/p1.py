from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import struct
import numpy as np

import tensorflow as tf
import sys
import inception_utils
slim = tf.contrib.slim

IMAGE_HEIGHT = 272
IMAGE_WIDTH = 512
NUM_LABELS = 16
NUM_CHANNELS = 3
# The random seed that defines initialization.
SEED = 42
CKPT_PATH = 'vtest/chkpt/saved.ckpt'

class Processor():
  def __init__(self,sess):
    self.proc,self.fd = self.inference()
    self.initSession(sess,CKPT_PATH)   

  def fiximg(self,img):
    fimg = img
    fimg[0:34,0:64,:] = np.ones([34,64,4],np.float32)
    fimg[0:34,448:512,:] = np.ones([34,64,4],np.float32)
    return fimg

  def block_reduction_a(self,inputs, scope=None, reuse=None, deep=1):
    """Builds Reduction-A block for Inception v4 network."""
    # By default use stride=1 and SAME padding
    # output depth is 64*deep + input depth
    with slim.arg_scope([slim.conv2d, slim.avg_pool2d, slim.max_pool2d],
                      stride=1, padding='SAME'):
      with tf.variable_scope(scope, 'BlockReductionA', [inputs], reuse=reuse):
        with tf.variable_scope('Branch_0'):
          branch_0 = slim.conv2d(inputs, 32*deep, [3, 3], stride=2,
                               #padding='VALID',
                               scope='RedA_1a_3x3')
        with tf.variable_scope('Branch_1'):
          branch_1 = slim.conv2d(inputs, 32*deep, [1, 1], scope='Conv2d_0a_1x1')
          branch_1 = slim.conv2d(branch_1, 32*deep, [3, 3], scope='Conv2d_0b_3x3')
          branch_1 = slim.conv2d(branch_1, 32*deep, [3, 3], stride=2,
                               #padding='VALID',
                               scope='Reduce_1a_3x3')
        with tf.variable_scope('Branch_2'):
          branch_2 = slim.max_pool2d(inputs, [3, 3], stride=2,
                                   #padding='VALID',
                                   scope='MaxPool_1a_3x3')
    return tf.concat(axis=3, values=[branch_0, branch_1, branch_2])

  def inception_v4_base(self, inputs, final_endpoint='Conv2d_4a_1x1', scope=None):
    """Creates the Inception V4 network up to the given final endpoint.
    Raises:
      ValueError: if final_endpoint is not set to one of the predefined values,
    """
    end_points = {}

    def add_and_check_final(name, net):
      end_points[name] = net
      return name == final_endpoint

    with tf.variable_scope(scope, 'InceptionV4base', [inputs]):
      with slim.arg_scope([slim.conv2d, slim.max_pool2d, slim.avg_pool2d],
                        stride=1, padding='SAME'):
        # 272 x 512 x 3
        net = slim.conv2d(inputs, 32, [3, 3], stride=2,
                        #padding='VALID',
                        scope='Conv2d_1aaa_3x3')
        if add_and_check_final('Conv2d_1aaa_3x3', net): return net, end_points
    
        # 136 x 256 x 32
        net = slim.conv2d(net, 32, [3, 3],
                        #padding='VALID',
                        scope='Conv2d_2a_3x3')
        if add_and_check_final('Conv2d_2a_3x3', net): return net, end_points

        # 2 x Reduction-A blocks with inputs:
        # 136 x 256 x 32 from above
        # 68 x 128 x (64*deep + inp) = 96
        
        for idx in range(2):
          block_scope = 'Mixed_3' + chr(ord('a') + idx)
          net = block_reduction_a(net, scope=block_scope, reuse=None, deep=idx+1)
          if add_and_check_final(block_scope, net): return net, end_points
    
        # "Fully connected" block
        # 34 x 64 x (64*2 + 96) = 224 input
        # 34 x 64 x 16 output
        net = slim.conv2d(net, 16, [1, 1],
                        #padding='VALID',
                        scope='Conv2d_4a_1x1')
        if add_and_check_final('Conv2d_4a_1x1', net): return net, end_points

    raise ValueError('Unknown final endpoint %s' % final_endpoint)

  def model(self,inputs, num_classes=NUM_LABELS, is_training=True,
                 dropout_keep_prob=0.8,
                 reuse=None,
                 scope='InceptionV4',
                 create_aux_logits=True):

    end_points = {}
    with tf.variable_scope(scope, 'InceptionV4model', [inputs], reuse=reuse) as scope:
      with slim.arg_scope([slim.batch_norm, slim.dropout],
                        is_training=is_training):
        net, end_points = inception_v4_base(inputs, scope=scope)

        with slim.arg_scope([slim.conv2d, slim.max_pool2d, slim.avg_pool2d],
                          stride=1, padding='SAME'):
          # Final pooling and prediction
          with tf.variable_scope('Logits'):
            logits = net
            end_points['Logits'] = logits
            #end_points['Predictions'] = tf.nn.softmax(net, name='Predictions')
    return logits, end_points

  def inference(self):
    process_data_node = tf.placeholder(tf.float32,
      shape=(1, IMAGE_HEIGHT, IMAGE_WIDTH, NUM_CHANNELS))
    process_model,_ = model(process_data_node,is_training=False,reuse=False)
    process_prediction = tf.nn.softmax(process_model)
    process_image = tf.argmax(process_prediction,dimension=3)
    return process_image,process_data_node

  def initSession(self,s,chk_file):
    s.as_default()
    init = tf.global_variables_initializer()
    s.run(init)

    saver = tf.train.Saver(tf.global_variables())
    saver.restore(s, chk_file)
    print("Restored model from %s" % (chk_file))

  def process(self,image):
    input_img = self.fiximg(image)
    result_img = self.sess.run(self.proc,feed_dict={self.fd: [input_img]})[0]
    return result_img
    



