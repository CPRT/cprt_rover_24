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
import time

from interfaces.srv import KeycapCmd
from sensor_msgs.msg import JointState

# import Jetson.GPIO as GPIO
# import interfaces.msg as GPIOmsg

matplotlib.use('tkagg')

#keyboard: 35.3 x 12.3 cm
#keycap: 1.2 x 1.4 cm
#reverse fisheye technique dimensions: 1280x720 pixels
#8.5 cm down to claw
#4.8 cm down the other way to claw
keyboard_dim = [35.3, 12.3, 1]
m_dim = [1280, 446, 1]

bs = [(-12.0, 2.5, 18.0), (-5.0, 0.3, 18.0), (-0.1, 0.1, 18.0), (-0.05, 0.05, 18.0)] #stands for, well, you know
bsCount = 0

colors = [(255,0,0), (229, 52, 235), (235, 85, 52),
          (14, 115, 51), (14, 115, 204)]

classes = {
  "a" : 0,
  "b" : 1,
  "c" : 2,
  "d" : 3,
  "e" : 4,
  "f" : 5,
  "g" : 6,
  "h" : 7,
  "i" : 8,
  "j" : 9,
  "k" : 10,
  "l" : 11,
  "m" : 12,
  "n" : 13,
  "o" : 14,
  "p" : 15,
  "q" : 16,
  "r" : 17,
  "s" : 18,
  "t" : 19,
  "u" : 20,
  "v" : 21,
  "w" : 22,
  "x" : 23,
  "y" : 24,
  "z" : 25,
  "caps_lock" : 26,
  "enter" : 27,
  "keyboard" : 28,
}

grid = {
  16 : (0, 0), #q
  22 : (1, 0), #w
  4 : (2, 0), #e
  17 : (3, 0), #r
  19 : (4, 0), #t
  24 : (5, 0), #y
  20 : (6, 0), #u
  8 : (7, 0), #i
  14 : (8, 0), #o
  15 : (9, 0), #p
  26 : (-1, 1), #caps lock
  0 : (0, 1), #a
  18 : (1, 1), #s
  3 : (2, 1), #d
  5 : (3, 1), #f
  6 : (4, 1), #g
  7 : (5, 1), #h
  9 : (6, 1), #j
  10 : (7, 1), #k
  11 : (8, 1), #l
  27 : (11, 1), #enter
  25 : (0, 2), #z
  23 : (1, 2), #x
  2 : (2, 2), #c
  21 : (3, 2), #v
  1 : (4, 2), #b
  13 : (5, 2), #n
  12 : (6, 2) #m
};

PATH_TO_CFG = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/exported-models/my_model/pipeline.config"
PATH_TO_CKPT = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/exported-models/my_model/checkpoint"
PATH_TO_LABELS = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo/annotations/label_map.pbtxt"
SHAFT = 7.95 #Ivan's camera mount shaft length (FIGURE OUT EXPERIMENTALLY)

#matplotlib.use('Qt5Agg')

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


class results():
    def __init__(self, data, tilt):
        self.data = data
        self.tilt = tilt
    
    def get_key_index(self, k):
        #get index from labels:
        ind = classes[k]
        
        for x in range(len(self.data["detection_classes"])):
            print(self.data["detection_classes"][x])
            if (self.data["detection_classes"][x] == ind):
                print("Found")
                return x
    
    def get_key_pos(self, k):
        ind = self.get_key_index(k)
        return self.data["detection_boxes"][ind];
        
    
    def get_likely_keycap_index(self):
        closest = 0
        mse = 10000 #stands for "mean sum of square error", or "distance" I suppose
        
        for x in range(len(self.data["detection_boxes"])):
            point_x = (self.data["detection_boxes"][x][1] + self.data["detection_boxes"][x][3])/2.0
            point_y = (self.data["detection_boxes"][x][0] + self.data["detection_boxes"][x][2])/2.0
            dist = math.sqrt((point_x-0.5)*(point_x-0.5) + (point_y-0.5)*(point_y-0.5)) + 0.5*(1-self.data["detection_scores"][x])
            if (dist < mse and self.data["detection_classes"][x] <= 25):
                mse = dist
                closest = x
        
        return closest
    
    def check_superior_overlaps(self, k):
        index = self.get_key_index(k)
        pos = self.data["detection_boxes"][index]
        
        pos_x = (pos[3]+pos[1])/2.0
        pos_y = (pos[2]+pos[0])/2.0
        
        for x in range(len(self.data["detection_boxes"])):
            if (x >= index):
                return False
            box = self.data["detection_boxes"][x]
            print(f'Index {index} with score {self.data["detection_scores"][index]} box {pos} {pos_x} {pos_y} compared to {x} {self.data["detection_scores"][x]} {box} {self.data["detection_classes"][x]}')
            if (pos_x > box[1] and pos_x < box[3] and pos_y > box[0] and pos_y < box[2] and self.data["detection_classes"][x] != classes["keyboard"]):
                return True
        
        return False
    
    def get_likely_keycap_pos(self): #returns the pixel dimensions of the keycap closest to the center
        closest = self.get_likely_keycap_index()
        
        print(f"Closest = {self.data['detection_boxes'][closest]} id {self.data['detection_classes'][closest]}")
        return self.data["detection_boxes"][closest]
    
    def dist_to_key(self, k):
        p = self.get_likely_keycap_pos()
        ratio_x = (p[3] - p[1]); #ratio of pixels to cm
        ratio_y = (p[2] - p[0]); #ratio of pixels to cm
        
        key = self.get_key_pos(k)
        
        center_x = (key[3]+key[1])/2.0
        center_y = (key[2]+key[0])/2.0
        
        p_x = (p[3]+p[1])/2.0
        p_y = (p[2]+p[0])/2.0
        
        if (self.check_superior_overlaps(k)):
            print("This key sucks!!! Gotta estimate I guess")
            ind = self.get_likely_keycap_index()
            ind = self.data['detection_classes'][ind]
            cx = grid[ind][0]
            cy = grid[ind][1]
            
            i2 = classes[k]
            tx = grid[i2][0]
            ty = grid[i2][1]
            
            center_x = p_x + (tx-cx)*ratio_x*1.25
            center_y = p_y + (ty-cy)*ratio_y*1.2
        else:
            print("This key is fine.")
        
        print(f"{p_x*1280} - {center_x*1280} {p_y*720} - {center_y*720} {ratio_x*1280} {ratio_y*720}")
        print(f"Thinks key {key} at {key[0]*720} {key[1]*1280} {key[2]*720} {key[3]*1280} with center {center_x*1280} {center_y*720}")
        print(f"Thinks center {p} at {p[0]*720} {p[1]*1280} {p[2]*720} {p[3]*1280} with center {p_x} {p_y}")
        print(f"Has {ratio_x*1280} pixels per 1.2 cm and {ratio_y*720} pixels per 1.4 cm")
        
        #dist_x = (center_x-p_x)*(1.2/ratio_x)
        #dist_y = (center_y-p_y)*(1.4/ratio_y)
        
        x_shift = SHAFT*math.sin(self.tilt) - 1.5# - 2.5 #supposed to be 1.5
        y_stick = SHAFT*math.cos(self.tilt)
        
        y_shift = SHAFT - y_stick
        
        print(f"Sticks: {x_shift} {y_shift}")
        
        #cam_center_x = 0.5 - ratio_x*1.5 + ratio_x*x_shift
        cam_center_x = 0.5# - ratio_x*x_shift
        cam_center_y = 0.5# - ratio_y*y_shift
        
        print(f"Thinks camera center at {cam_center_x*1280} {cam_center_y*720}")
        
        dist_x = (center_x-cam_center_x)*(1.2/ratio_x)
        dist_y = (center_y-cam_center_y)*(1.4/ratio_y)
        
        d_x = ratio_x*1280*ratio_y*720
        dist_z = 34.2 + -0.00597*d_x + 0.000000436*d_x*d_x
        
        print(f"Old dists: {dist_x} {dist_y}")
        print(f"New dists: {dist_x+x_shift} {dist_y-y_shift}")
        return (dist_x+x_shift, dist_y-y_shift, dist_z)
    
    #def center_to_key(self, k)

#IMAGE_PATHS = ["../../images/keyboard2.jpg", "../../images/test3.jpg", "../../images/test4.jpg", "../../images/test5.jpg", "../../images/test6.jpg"]
#image_path = "/data_disk/will/python/TensorFlow/workspace/keycap_demo2/keycap_demo//images/keyboard.jpg"
#image_path = "/home/will/cprt_rover_24/key.jpg"
image_path = "/home/will/cprt_rover_24/key4.jpg"

class tf2Keyboard(Node):
    def __init__(self):
        super().__init__("tfKeyboard")
        self.srv = self.create_service(KeycapCmd, "keycap_cmd", self.keyboard_callback)
        
        self.tilt = 0;
        self.wristtilt_sub = self.create_subscription(
            JointState(),
            "/joint_states",
            self.joint_callback,
            10,
        )
        
        '''self.get_logger().info("Hello world!")
        print('Running inference for {}... '.format(image_path), end='')

        image_np = load_image_into_numpy_array(image_path)

        # Things to try:
        # Flip horizontally
        # image_np = np.fliplr(image_np).copy()

        # Convert image to grayscale
        # image_np = np.tile(
        #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

        input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)

        detections = detect_fn(input_tensor)
        print(detections)

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

        viz_utils.visualize_boxes_and_labels_on_image_array(
                image_np_with_detections,
                detections['detection_boxes'],
                detections['detection_classes']+label_id_offset,
                detections['detection_scores'],
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=29,
                min_score_thresh=.00,
                agnostic_mode=False)

        print('Done')
        result = results(detections)
        print(result.get_key_index("q"))
        print(f"{result.get_key_pos('q')} key pos")
        print(result.dist_to_key("q"))
        #print(detections['detection_classes']['q'])
        #print(detections['detection_scores']['q'])
        plt.figure()
        plt.imshow(image_np_with_detections)
        plt.show()#'''
        
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

    def joint_callback(self, msg):
        self.tilt = msg.position[5]

    def keyboard_callback(self, request, response):
        '''global bsCount
        response.x = bs[bsCount][0]
        response.y = bs[bsCount][1]
        response.z = bs[bsCount][2]
        
        bsCount += 1
        if (bsCount >= len(bs)):
          bsCount = 0
        
        return response'''
        
        
        cap = cv2.VideoCapture("/dev/video4")  # 0 is the default camera (usually the built-in one)
        #cap = cv2.VideoCapture(0)  # 0 is the default camera (usually the built-in one)
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

        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        image_np = np.array(frame)
        time.sleep(2.5) #wait for camera to stop shaking IRL
        #image = Image.open(image_path)
        
        #image_np = np.array(image)
        
        #plt.imshow(image_np)
        #plt.show()
        
        #What I desired from Tensorflow was a "model" to adapt to your infinite rotation. At first, TensorFlow altered the properties 
        #of its own degrees of freedom to bypass your slack, but I was unable to do so. However, once TensorFlow adapts to an
        #attack, its analysis continues. The second adapatation was exactly what I had been anticipating, but its target wasn't the
        #slack. By subscribing to the /joint_states topic and using the sine and cosine law to produce linear offsets caused by the
        #camera mount, it created a rotational matrix that targets the world itself. You were magnificent, mech team, I shall never forget
        #you as long as I live.
        center = (640, 360)
        angle = -self.tilt/(2*math.pi)*360
        print(f"Angle = {angle}")
        world_rotating_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        image_np = cv2.warpAffine(image_np, world_rotating_matrix, np.array([1280, 720]))
        
        #plt.imshow(image_np)
        #plt.show()
        
        self.get_logger().info("Detecting keys, please wait")
        
        input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)

        detections = detect_fn(input_tensor)

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
        
        result = results(detections, self.tilt)
        key = result.dist_to_key(request.key)
        print(key)

        viz_utils.visualize_boxes_and_labels_on_image_array(
                image_np_with_detections,
                detections['detection_boxes'],
                detections['detection_classes']+label_id_offset,
                detections['detection_scores'],
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=29,
                min_score_thresh=.30,
                agnostic_mode=False)

        plt.imshow(image_np_with_detections)
        print('Done')
        plt.show()
        
        response.x = key[0]
        response.y = -key[1]
        response.z = key[2]
        
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

