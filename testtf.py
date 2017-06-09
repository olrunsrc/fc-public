import tensorflow as tf
hello = tf.constant("Tensorflow is working")
sess = tf.Session()
print(sess.run(hello))

