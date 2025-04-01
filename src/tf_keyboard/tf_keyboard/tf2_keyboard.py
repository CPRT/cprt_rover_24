from math import radians
import threading
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String

import rclpy.time

import cv2
import time
import glob
import numpy as np
import tensorflow as tf
from PIL import Image
import matplotlib.pyplot as plt
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder
import os
import matplotlib
import math

from interfaces.srv import KeycapCmd

# import Jetson.GPIO as GPIO
# import interfaces.msg as GPIOmsg

#keyboard: 35.3 x 12.3 cm
#keycap: 1.2 x 1.4 cm
keyboard_dim = [35.3, 12.3, 1]
m_dim = [1280, 446, 1]

colors = [(255,0,0), (229, 52, 235), (235, 85, 52),
          (14, 115, 51), (14, 115, 204)]

PATH_TO_CFG = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/exported-models/my_model/pipeline.config"
PATH_TO_CKPT = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/exported-models/my_model/checkpoint"
PATH_TO_LABELS = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/annotations/label_map.pbtxt"

matplotlib.use('Qt5Agg')

print('Loading model... ', end='')
start_time = time.time()

# Load pipeline config and build a detection model
configs = config_util.get_configs_from_pipeline_file(PATH_TO_CFG)
model_config = configs['model']
detection_model = model_builder.build(model_config=model_config, is_training=False)

# Restore checkpoint
ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
ckpt.restore(os.path.join(PATH_TO_CKPT, 'ckpt-0')).expect_partial()

@tf.function
def detect_fn(image):
    """Detect objects in image."""

    image, shapes = detection_model.preprocess(image)
    prediction_dict = detection_model.predict(image, shapes)
    detections = detection_model.postprocess(prediction_dict, shapes)

    return detections

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

def load_image_into_numpy_array(path):
    """Load an image from file into a numpy array.

    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.

    Args:
      path: the file path to the image

    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    return np.array(Image.open(path))


#IMAGE_PATHS = ["../../images/keyboard2.jpg", "../../images/test3.jpg", "../../images/test4.jpg", "../../images/test5.jpg", "../../images/test6.jpg"]
IMAGE_PATHS = ["../../images/keyboard.jpg"]

class tf2Keyboard(Node):
    def __init__(self):
        super().__init__("tfKeyboard")

        # GPIO.setmode(GPIO.BOARD)
        # output_pins = {
        #     'JETSON_XAVIER': 18,
        #     'JETSON_NANO': 33,
        #     'JETSON_NX': 33,
        #     'CLARA_AGX_XAVIER': 18,
        #     'JETSON_TX2_NX': 32,
        #     'JETSON_ORIN': 18,
        #     'JETSON_ORIN_NX': 33,
        #     'JETSON_ORIN_NANO': 33
        # }
        # output_pin = output_pins.get(GPIO.model, None)
        # if output_pin is None:
        #     raise Exception('PWM not supported on this board')
        

        # GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        # self.gripper = GPIO.PWM(output_pin, 50)
        
        #self.joystick = self.create_subscription(
         #   Joy, "/joystick/arm", self.joy_callback, 5)
        self.srv = self.create_service(KeycapCmd, "keycap_cmd", self.keyboard_callback)
        
        
        self.get_logger().info("Hello world!")
        
        '''cap = cv2.VideoCapture("/dev/video4")  # 0 is the default camera (usually the built-in one)
        if not cap.isOpened():
          print("Error: Could not access the camera.")
          exit()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        ret, frame = cap.read()
        cap.release()
        
        #cursed fisheye reversal
        dim = (1280, 720)
        k = np.array([[817.1125488944084, 0.0, 712.3596465245281], [0.0, 758.2820572597227, 400.5753281200634], [0.0, 0.0, 1.0]])
        d = np.array([[-0.17895858184738406], [0.38653318939991055], [-0.7363420182211281], [0.4482813189308249]])
        
        h,w = frame.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), k, dim, cv2.CV_16SC2)
        frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        
        #cv2.imshow('Camera', frame)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        origFrame = frame
        nparray = np.array(frame)
        plt.imshow(nparray)
        plt.show()
        
        with tf.Session(graph = tf.Graph()) as sess:
          tf.saved_model.loader.load(sess, ['serve'], model_path)
          graph = tf.get_default_graph()
          input_tensor = graph.get_tensor_by_name("serving_default_input_tensor:0")
          det_score = graph.get_tensor_by_name("StatefulPartitionedCall:6")
          det_class = graph.get_tensor_by_name("StatefulPartitionedCall:2")
          det_boxes = graph.get_tensor_by_name("StatefulPartitionedCall:0")
          det_numbs = graph.get_tensor_by_name("StatefulPartitionedCall:7")
          det_keypoint = graph.get_tensor_by_name("StatefulPartitionedCall:4")
          det_keypoint_score = graph.get_tensor_by_name("StatefulPartitionedCall:3")
          
          frame = origFrame
          height, width, _ = frame.shape
          image_exp_dims = np.expand_dims(frame, axis=0)
          start_time = time.time()
          score,classes,boxes,nums_det, \
          keypoint,keypoint_score = sess.run([det_score, det_class, det_boxes, 
                                              det_numbs,det_keypoint,det_keypoint_score], 
                                              feed_dict={input_tensor:image_exp_dims})
          print(f"Height {height} width {width}")
          for i in range(int(nums_det[0])):
              if(score[0][i]*100 > 50): 
                  per_box = boxes[0][i]
                  y1 = int(per_box[0]*height)
                  x1 = int(per_box[1]*width)
                  y2 = int(per_box[2]*height)
                  x2 = int(per_box[3]*width)
      
                  p1 = (x1,y1)
                  p2 = (x2,y2)
                  cv2.rectangle(frame, p1, p2, (0,255,0), 3)
          labeled = process_keypoint(keypoint[0][0], keypoint_score[0], height, width, frame)
          #cv2.imshow("display",frame)
          nparray = np.array(labeled)
          print("Time: ", time.time() - start_time)
          plt.imshow(frame)
          #plt.show()
          
          #perspective transformation
          l = []
          for i, kp in enumerate(keypoint[0][0]):
              l.append([kp[1]*width, kp[0]*height])
          #print(l)
          topd = math.sqrt((l[1][0] - l[0][0])*(l[1][0] - l[0][0]) + (l[1][1] - l[0][1])*(l[1][1] - l[0][1]))
          botd = math.sqrt((l[2][0] - l[3][0])*(l[2][0] - l[3][0]) + (l[2][1] - l[3][1])*(l[2][1] - l[3][1]))
          print("Top length (pixels): "+str(topd));
          print("Top distance (estimated) "+str(92.4*math.pow(math.e, -0.00116*topd)))
          print("Bot length: "+str(botd))
          print("Bot dist (estimated) "+str(92.4*math.pow(math.e, -0.00116*botd)))
          
          src = np.float32(l);
          dest = np.float32([[0, 0], [1280, 0], [1280, 446], [0, 446]])
          matrix = cv2.getPerspectiveTransform(src, dest)
          
          #Perspective technique amplification: blue
          blue = np.dot(matrix, np.array([width/2, height/2, 1]))
          blue[0] /= blue[2]
          blue[0] /= blue[2]
          
          res = cv2.warpPerspective(frame, matrix, (1280, 446))
          
          for k, p in key_pos_dict.items():
              cv2.circle(res,(p[0], p[1]),5,colors[1],-1)
              
              #Perspective technique reversal: red
              x, inv = cv2.invert(matrix)
              orig = np.array(p, dtype=np.float32)
              
              red = np.dot(inv, orig)
              red[0] /= red[2]
              red[1] /= red[2]
              cv2.circle(frame,(int(red[0]), int(red[1])),5,colors[1],-1)
          
          red = key_pos_dict["q"]
          
          purple = red.copy()
          for x in range(0, len(purple)):
              print(f"{x} {red[x]} - {blue[x]} = {red[x] - blue[x]}, {(red[x] - blue[x])/m_dim[x]*keyboard_dim[x]}")
              purple[x] -= blue[x]
              purple[x] = purple[x]/m_dim[x] * keyboard_dim[x]
          
          cv2.circle(res,(int(blue[0]), int(blue[1])),5,colors[2],-1)
          print(f"Purple {purple}")
          plt.imshow(res)
          plt.show()
          plt.imshow(frame)
          plt.show()''' #it could be useful later no cope
          
    def keyboard_callback(self, request, response):
        response.x = 5.0
        response.y = 6.0
        response.z = 15.0
        return response
        
        '''cap = cv2.VideoCapture("/dev/video4")  # 0 is the default camera (usually the built-in one)
        if not cap.isOpened():
          print("Error: Could not access the camera.")
          exit()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        ret, frame = cap.read()
        cap.release()
        
        #cursed fisheye reversal
        dim = (1280, 720)
        k = np.array([[817.1125488944084, 0.0, 712.3596465245281], [0.0, 758.2820572597227, 400.5753281200634], [0.0, 0.0, 1.0]])
        d = np.array([[-0.17895858184738406], [0.38653318939991055], [-0.7363420182211281], [0.4482813189308249]])
        
        h,w = frame.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), k, dim, cv2.CV_16SC2)
        frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        
        #cv2.imshow('Camera', frame)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        origFrame = frame
        nparray = np.array(frame)
        plt.imshow(nparray)
        plt.show()
        
        with tf.Session(graph = tf.Graph()) as sess:
          tf.saved_model.loader.load(sess, ['serve'], model_path)
          graph = tf.get_default_graph()
          input_tensor = graph.get_tensor_by_name("serving_default_input_tensor:0")
          det_score = graph.get_tensor_by_name("StatefulPartitionedCall:6")
          det_class = graph.get_tensor_by_name("StatefulPartitionedCall:2")
          det_boxes = graph.get_tensor_by_name("StatefulPartitionedCall:0")
          det_numbs = graph.get_tensor_by_name("StatefulPartitionedCall:7")
          det_keypoint = graph.get_tensor_by_name("StatefulPartitionedCall:4")
          det_keypoint_score = graph.get_tensor_by_name("StatefulPartitionedCall:3")
          
          frame = origFrame
          height, width, _ = frame.shape
          image_exp_dims = np.expand_dims(frame, axis=0)
          start_time = time.time()
          score,classes,boxes,nums_det, \
          keypoint,keypoint_score = sess.run([det_score, det_class, det_boxes, 
                                              det_numbs,det_keypoint,det_keypoint_score], 
                                              feed_dict={input_tensor:image_exp_dims})
          print(f"Height {height} width {width}")
          for i in range(int(nums_det[0])):
              if(score[0][i]*100 > 50): 
                  per_box = boxes[0][i]
                  y1 = int(per_box[0]*height)
                  x1 = int(per_box[1]*width)
                  y2 = int(per_box[2]*height)
                  x2 = int(per_box[3]*width)
      
                  p1 = (x1,y1)
                  p2 = (x2,y2)
                  cv2.rectangle(frame, p1, p2, (0,255,0), 3)
          labeled = process_keypoint(keypoint[0][0], keypoint_score[0], height, width, frame)
          #cv2.imshow("display",frame)
          nparray = np.array(labeled)
          print("Time: ", time.time() - start_time)
          #plt.imshow(frame)
          #plt.show()
          
          #perspective transformation
          l = []
          for i, kp in enumerate(keypoint[0][0]):
              l.append([kp[1]*width, kp[0]*height])
          #print(l)
          topd = math.sqrt((l[1][0] - l[0][0])*(l[1][0] - l[0][0]) + (l[1][1] - l[0][1])*(l[1][1] - l[0][1]))
          botd = math.sqrt((l[2][0] - l[3][0])*(l[2][0] - l[3][0]) + (l[2][1] - l[3][1])*(l[2][1] - l[3][1]))
          print("Top length (pixels): "+str(topd));
          print("Top distance (estimated) "+str(92.4*math.pow(math.e, -0.00116*topd)))
          print("Bot length: "+str(botd))
          print("Bot dist (estimated) "+str(92.4*math.pow(math.e, -0.00116*botd)))
          dis = ((92.4*math.pow(math.e, -0.00116*topd))+(92.4*math.pow(math.e, -0.00116*botd)))/2.0
          
          src = np.float32(l);
          dest = np.float32([[0, 0], [1280, 0], [1280, 446], [0, 446]])
          matrix = cv2.getPerspectiveTransform(src, dest)
          
          #Perspective technique amplification: blue
          blue = np.dot(matrix, np.array([width/2, height/2, 1]))
          blue[0] /= blue[2]
          blue[0] /= blue[2]
          
          res = cv2.warpPerspective(frame, matrix, (1280, 446))
          
          for k, p in key_pos_dict.items():
              cv2.circle(res,(p[0], p[1]),5,colors[1],-1)
              
              #Perspective technique reversal: red
              x, inv = cv2.invert(matrix)
              orig = np.array(p, dtype=np.float32)
              
              red = np.dot(inv, orig)
              red[0] /= red[2]
              red[1] /= red[2]
              cv2.circle(frame,(int(red[0]), int(red[1])),5,colors[1],-1)
          
          data = str(request.key)
          print(f"Data {data}")
          red = key_pos_dict[data] #please do not send anything other than invididual lowercase letters, I don't know what happens otherwise but it can't be good.
          
          purple = red.copy()
          for x in range(0, len(purple)):
              print(f"{x} {red[x]} - {blue[x]} = {red[x] - blue[x]}, {(red[x] - blue[x])/m_dim[x]*keyboard_dim[x]}")
              purple[x] -= blue[x]
              purple[x] = purple[x]/m_dim[x] * keyboard_dim[x]
          
          cv2.circle(res,(int(blue[0]), int(blue[1])),5,colors[2],-1)
          print(f"Purple {purple}")
          
          response.x = purple[0]
          response.y = purple[1]
          response.z = dis
          return response'''
      


def main(args=None):
    rclpy.init(args=args)
    node = tf2Keyboard()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

