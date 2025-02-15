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
import tensorflow.compat.v1 as tf
from PIL import Image
import matplotlib.pyplot as plt
import os
import matplotlib
import math

from interfaces.srv import KeycapCmd

# import Jetson.GPIO as GPIO
# import interfaces.msg as GPIOmsg

model_path = "/data_disk/will/python/TensorFlow/workspace/corner_demo/exported-models/corner_model/saved_model"

#keyboard: 35.3 x 12.3 cm
#keycap: 1.2 x 1.4 cm
keyboard_dim = [35.3, 12.3, 1]
m_dim = [1280, 446, 1]

colors = [(255,0,0), (229, 52, 235), (235, 85, 52),
          (14, 115, 51), (14, 115, 204)]

#cv2.namedWindow("display", cv2.WINDOW_NORMAL)
def process_keypoint(kp, kp_s, h, w, img):
    for i, kp_data in enumerate(kp):
        cv2.circle(img,(int(kp_data[1]*w), int(kp_data[0]*h)),5,colors[i],-1)
    return img

key_pos_dict = {"q" : [133, 190, 1],
    "w" : [204, 190, 1],
    "e" : [275, 190, 1],
    "r" : [347, 190, 1],
    "t" : [418, 190, 1],
    "y" : [492, 190, 1],
    "u" : [562, 190, 1],
    "i" : [635, 190, 1],
    "o" : [706, 190, 1],
    "p" : [776, 190, 1],
    "a" : [151, 266, 1],
    "s" : [222, 266, 1],
    "d" : [292, 266, 1],
    "f" : [362, 266, 1],
    "g" : [435, 266, 1],
    "h" : [509, 266, 1],
    "j" : [579, 266, 1],
    "k" : [655, 266, 1],
    "l" : [724, 266, 1],
    "z" : [186, 335, 1],
    "x" : [257, 335, 1],
    "c" : [328, 335, 1],
    "v" : [400, 335, 1],
    "b" : [471, 335, 1],
    "n" : [544, 335, 1],
    "m" : [616, 335, 1],
}

class tfKeyboard(Node):
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
        response.x = 1.0
        response.y = 2.0
        response.z = 25.0
        return response
        
        cap = cv2.VideoCapture("/dev/video4")  # 0 is the default camera (usually the built-in one)
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
          return response
      


def main(args=None):
    rclpy.init(args=args)
    node = tfKeyboard()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

