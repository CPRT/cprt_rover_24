from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time

import tensorflow as tf
import time
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import os
import matplotlib

# import Jetson.GPIO as GPIO
# import interfaces.msg as GPIOmsg

#load random tensorflow things
PATH_TO_CFG = "/home/will/2023Rover/keycap_demo/exported-models/my_model/pipeline.config"
PATH_TO_CKPT = "/home/will/2023Rover/keycap_demo/exported-models/my_model/checkpoint"
PATH_TO_LABELS = "/home/will/2023Rover/keycap_demo/annotations/label_map.pbtxt"

matplotlib.use('Qt5Agg')

print('Loading model... ', end='')
start_time = time.time()

class tfKeyboard(Node):
  def __init__(self):
    super().__init__("tfKeyboard")
    print('Loading model... ', end='')
    start_time = time.time()

    # Load pipeline config and build a detection model
    self.configs = config_util.get_configs_from_pipeline_file(PATH_TO_CFG)
    self.model_config = self.configs['model']
    self.detection_model = model_builder.build(model_config=self.model_config, is_training=False)

    # Restore checkpoint
    self.ckpt = tf.compat.v2.train.Checkpoint(model=self.detection_model)
    self.ckpt.restore(os.path.join(PATH_TO_CKPT, 'ckpt-0')).expect_partial()
    
    
    #print loading time
    end_time = time.time()
    elapsed_time = end_time - start_time
    print('Done! Took {} seconds'.format(elapsed_time))
    
    self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
    
    #run a thing
    self.infer("/home/will/2023Rover/keycap_demo/images/keyboard2.jpg")
  
  @tf.function
  def detect_fn(self, image):
    """Detect objects in image."""

    image, shapes = self.detection_model.preprocess(image)
    prediction_dict = self.detection_model.predict(image, shapes)
    detections = self.detection_model.postprocess(prediction_dict, shapes)

    return detections
  
  
  def load_image_into_numpy_array(self, path):
    return np.array(Image.open(path))

  def infer(self, image_path):
    print('Running inference for {}... '.format(image_path), end='')

    image_np = self.load_image_into_numpy_array(image_path)

    # Things to try:
    # Flip horizontally
    # image_np = np.fliplr(image_np).copy()

    # Convert image to grayscale
    # image_np = np.tile(
    #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

    input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)

    detections = self.detect_fn(input_tensor)

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    label_id_offset = 1
    image_np_with_detections = image_np.copy()
    
    print(detections['detection_classes'])
    print(detections['detection_scores'])
    print(detections['detection_boxes'])

    viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections['detection_boxes'],
            detections['detection_classes']+label_id_offset,
            detections['detection_scores'],
            self.category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.30,
            agnostic_mode=False)

    plt.figure()
    plt.imshow(image_np_with_detections)
    print('Done')
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = tfKeyboard()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

