
# coding: utf-8

# In[1]:

get_ipython().magic(u'matplotlib inline')

import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpi

def fiximg(img):
    fimg = img
    s34 = (img[0:34,448:512,2]*16.0 + 0.5).astype(int)
    fimg[0:34,0:64,:] = np.ones([34,64,4],np.float32)
    fimg[0:34,448:512,:] = np.ones([34,64,4],np.float32)
    s34[0:4,0:8] = 0.0
    s34[0:4,56:64] = 0.0
    return (fimg,s34)

IMAGE_ROOT='../images'

def extract_data(mapname,imno):
  try:
    fn = "none"
    data = np.empty([272,512,3],np.float32)
    fn = IMAGE_ROOT+'/{0:s}/{0:s}_{1:05d}.png'.format(mapname,imno)
    img,s34 = fiximg(mpi.imread(fn))
    data = img[:,:,0:3]
  except Exception as e:
    print("Filename %s error %s"%(fn, e))  
  return data,s34

implot = plt.imshow(extract_data('jun004',77)[0])


# In[2]:

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

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

def block_reduction_a(inputs, scope=None, reuse=None, deep=1):
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


def inception_v4_base(inputs, final_endpoint='Conv2d_4a_1x1', scope=None):
  """Creates the Inception V4 network up to the given final endpoint.
  Args:
    inputs: a 4-D tensor of size [batch_size, height, width, 3].
    final_endpoint: specifies the endpoint to construct the network up to.
      It can be one of [ 'Conv2d_1a_3x3', 'Conv2d_2a_3x3', 
      'Mixed_3a', 'Mixed_3b',
      'Conv2d_4a_1x1']
    scope: Optional variable_scope.
  Returns:
    logits: the logits outputs of the model.
    end_points: the set of end_points from the inception model.
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


def model(inputs, num_classes=16, is_training=True,
                 dropout_keep_prob=0.8,
                 reuse=None,
                 scope='InceptionV4',
                 create_aux_logits=True):
  """Creates the Inception V4 model.
  Args:
    inputs: a 4-D tensor of size [batch_size, height, width, 3].
    num_classes: number of predicted classes.
    is_training: whether is training or not.
    dropout_keep_prob: float, the fraction to keep before final layer.
    reuse: whether or not the network and its variables should be reused. To be
      able to reuse 'scope' must be given.
    scope: Optional variable_scope.
    create_aux_logits: Whether to include the auxiliary logits.
  Returns:
    logits: the logits outputs of the model.
    end_points: the set of end_points from the inception model.
  """
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

print('Done')


# In[3]:

process_data_node = tf.placeholder(tf.float32,
  shape=(1, IMAGE_HEIGHT, IMAGE_WIDTH, NUM_CHANNELS))
process_model,_ = model(process_data_node,is_training=False,reuse=False)
process_prediction = tf.nn.softmax(process_model)
process_image = tf.argmax(process_prediction,dimension=3)
print('Done')


# In[4]:

# Create a new interactive session that we'll use in
# subsequent code cells.
s = tf.InteractiveSession()

# Use our newly created session as the default for 
# subsequent operations.
s.as_default()

# Initialize all the variables we defined above.
init = tf.global_variables_initializer()

s.run(init)
print('Done')


# In[5]:

saver = tf.train.Saver(tf.global_variables())
checkpoint_path = 'chkpt/model2875.ckpt-2875'
saver.restore(s, checkpoint_path)
print("Restored model from %s" % (checkpoint_path))


# In[8]:

def plottruth(imdata,truedat,rawdat):
  imdata[0,0:16]=np.arange(16.0)
  truedat[0,0:16]=np.arange(16.0)
  fig, axs = plt.subplots(2, 2, figsize=(16,6))
  d1 = axs[0,0].imshow(imdata)
  axs[0,0].set_xlabel('Predicted')
  fig.colorbar(d1, ax=axs[0,0])
  axs[0,1].hist(imdata.flatten(), bins=16, range=(0.0,15.9), fc='r', ec='r')
  axs[0,1].set_xlabel('Predicted (Hist)')
  d2 = axs[1,0].imshow(truedat)
  axs[1,0].set_xlabel('Ground Truth')
  fig.colorbar(d2, ax=axs[1,0])
  axs[1,1].imshow(rawdat)
  axs[1,1].set_xlabel('Raw Image')
  plt.show()
    
def findtruth(imno):
  input_img,true_img = extract_data('jun004',imno)
  result_img = s.run(process_image,feed_dict={process_data_node: [input_img]})[0]
  plottruth(result_img,true_img,input_img)
    
findtruth(3)


# In[ ]:



