import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """

    # MODEL IS NOT FROZEN, SO WEIGHTS CAN BE CHANGED
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph() # get graph object

    # get tensors from vgg16 layers
    # vgg16 has already learned from imagenet and we will use it as our encoder
    # the vgg16 has pooling layers and then an input that goes into a fully connected layer
    input_image = graph.get_tensor_by_name(vgg_input_tensor_name)
    # allows us to choose a keep probability, allowing us to choose how much data we want to throw away to avoid overfitting
    keep_prob = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3 = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4 = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7 = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)
    
    return input_image, keep_prob, layer3, layer4, layer7
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """

    # add 1x1 convolution on top of VGG-16 to reduce number of filters from 4096 to the number of classes for our specific model
    # in this example we only have 2 classes for pixels, road or not road
    tf.Print(layer7_out, [tf.shape(vgg_layer7_out]))
    layer7_conv = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(layer7_conv, [tf.shape(layer7_conv]))

    # upsample to size of layer 4
    # stride of 2 is what is upsampling by 2
    # padding of same to keep the same img size as the input
    # shape of final convolutional layer output is 4-Dimensional: (batch_size, original_height, original_width, num_classes)
    layer4_upsmp7 = tf.layers.conv2d_transpose(layer7_conv, num_classes, 4, strides=(2,2), padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(layer4_upsmp7, [tf.shape(layer4_upsmp7]))

    # 1x1 conv to reduce classes
    layer4_conv = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(layer4_conv, [tf.shape(layer4_conv]))

    # Skip Connections combine the output of layers further back in the network
    # Layer 7 was required to be upsampled in order to be the same size as layer 4

    layer4out = tf.add(layer4_upsmp7, layer4_conv )
    tf.Print(layer4out, [tf.shape(layer4out]))

    # upsample to size of layer 3
    layer3_upsmp4 = tf.layers.conv2d_transpose(layer4out, num_classes, 4, strides=(2,2), padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(layer3_upsmp4, [tf.shape(layer3_upsmp4]))

    # 1x1 conv to reduce classes
    layer3_conv = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(layer3_conv, [tf.shape(layer3_conv]))

    layer3out = tf.add(layer3_upsmp4, layer3_conv)
    tf.Print(layer3out, [tf.shape(layer3out]))

    # upsample to size of orig image
    nn_last_layer = tf.layers.conv2d_transpose(layer3out, num_classes, 16, strides=(8,8), padding='same', kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    tf.Print(nn_last_layer, [tf.shape(nn_last_layer]))

    return nn_last_layer
tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """

    # Define loss so we can approach training the FCN just like a normal classification CNN

    # output layer tensor is 4D so we reshape to 2D
    logits = tf.reshape(nn_last_layer, (-1, num_classes))

    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits, labels=correct_label))

    # use Adam Optimizer due to less hyper paramerters
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    train_op = optimizer.minimize(cross_entropy_loss)


    return logits, train_op, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # TODO: Implement function

    for i in range(epochs):
        print("Epoch {f}".format(i))
        for image, label in get_batches_fn(batch_size):
            # training
            _, loss = sess.run([train_op, cross_entropy_loss], feed_dict={input_image: image, correct_label: label, keep_prob: 0.5, learning_rate: 0.0009})
            print("Loss: {:.3f}".format(loss))
        print("Next Epoch")

tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    # Parameters
    epochs = 10
    batch_size = 5

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        correct_label = tf.placeholder(tf.init32, [None, None, None, num_classes], name='correct_label')
        learning_rate = tf.placeholder(tf.float32, name='learning_rate')


        input_image, keep_prob, layer3, layer4, layer7 = load_vgg(sess, vgg_path)
        nn_last_layer = layers(layer3, layer4, layer7, num_classes)
        logits, train_op, cross_entropy_loss = optimize(nn_last_layer, correct_label, learning_rate, num_classes)

        # Train NN using the train_nn function
        train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image, correct_label, keep_prob, learning_rate )

        # Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
