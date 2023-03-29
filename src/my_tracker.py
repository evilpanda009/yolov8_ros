#!/usr/bin/env python3
import rospy
#from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, Joy
from yolov8_ros.msg import Box,Boxes
import time
import random
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

class Isolate():
    def __init__(self):
        self.box_sub = rospy.Subscriber('/yolov8_boxes', Boxes, self.box_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.lidar_sub = rospy.Subscriber('/obstacle', Int32, self.lidar_callback)
        # self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        #self.depth_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.depth_callback)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.image_width = 0
        # self.image_height = 0
        self.boxes = Boxes()
        # self.image = []
        self.subject_depth = -1
        self.cx_old = 0
        self.center = 320
        self.linear_vel = 0.5
        self.angular_vel = 0.009
        self.bb = []
        self.subject = Box()
        self.count = 0
        self.det = 0
        self.bb_area = 0
        self.img_area = 480*640
        self.keep_moving = True
        self.forward = True
        self.bridge = CvBridge()
        self.kill = False
        
    def lidar_callback(self, data):
        if data.data == 1:
            self.forward = False
        else:
           self.forward = True

    def joy_callback(self, data):
        if data.buttons[3] == 1:
            self.stop()
            self.keep_moving = False
            self.kill = True
        elif data.buttons[4] == 1:
            self.box_sub.unregister()
            self.stop()
            self.keep_moving = False
            self.kill = True
            
        else:
            self.kill = False
        if data.buttons[5] == 1:
            self.box_sub = rospy.Subscriber('/yolov8_boxes', Boxes, self.box_callback)
        

    
    def move_forward(self):
        msg = Twist()
        speed = self.linear_vel * 640/self.subject.width if self.subject is not None and not self.kill and self.forward else 0.0
        msg.linear.y = 0.0
        msg.linear.x = speed if speed < 2.0 else 2.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.pub_vel.publish(msg)
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.pub_vel.publish(msg)

    def rotate_with_linear(self, rotation):
        msg = Twist()
        msg.linear.x =0.0# self.linear_vel if self.forward else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = rotation* self.angular_vel if abs(rotation)>30 else 0.0
        self.pub_vel.publish(msg)

    def get_cx_cy(self):
        if len(self.bb)!=0:
            xmin = self.subject.top_left_x
            ymin = self.subject.top_left_y
            xmax = xmin + self.subject.width
            ymax = ymin + self.subject.height
            self.cx_old = int((xmin + xmax)/2)

    def lets_move(self):
        if len(self.bb)>0:    
            
            self.get_cx_cy()
            error = float(self.center - self.cx_old)
            # while(abs(error) > 30 and not rospy.is_shutdown() and self.keep_moving):
            #         self.get_cx_cy()                  
            #         error = float(self.center - self.cx_old)
            #         rotation = error if (error>0) else -error # decides direction of rotation
            #         rotation_time = error/100
            #         self.rotate_with_linear(rotation, True)
              
            # #self.stop()
            # while not rospy.is_shutdown() and self.keep_moving:
            #     self.move_forward()
            # self.stop()

            while not rospy.is_shutdown() and self.keep_moving and abs(error) > 30:
                self.get_cx_cy()
                error = float(self.center - self.cx_old)
                rotation = error  # decides direction of rotation
                if not self.kill:
                    self.rotate_with_linear(rotation)
            while not rospy.is_shutdown() and self.keep_moving:
                self.move_forward()
            self.stop()

    


    def box_callback(self,msg):
        #print("here")
        if(len(msg.Boxes)>0):
            self.boxes = msg.Boxes 
            self.bb = self.boxes
            self.subject = self.bb[0] 
            self.bb_area = self.subject.width*self.subject.height
            self.det += 1
            self.count = 0
            if(self.det>3):
                self.keep_moving = True
                self.det = 0
            if(self.bb_area > 0.8*self.img_area or self.subject.width > 0.6*640):
                self.forward = False
            else:
                self.forward = True

                  # Write service here to select any ID instead of 0
        else:
            self.boxes = []
            self.bb = self.boxes
            self.subject = None
            self.count += 1
            self.det = 0
            if self.count > 10:
                self.keep_moving = False
                self.count = 0
            

    # def depth_callback(self, msg):
    #     print("here") 
    #     self.depths = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
    #     self.image_height = msg.height
    #     self.image_width = msg.width
    #     self.center = self.image_width/2
    #     self.record_depth()

    # def image_callback(self, msg):
    #     self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
    #     self.image_height = msg.height
    #     self.image_width = msg.width
    #     self.center = self.image_width/2
    #     self.img_area = self.image_height*self.image_width
    #     if self.bb_area > 0.8*self.img_area:
    #         self.keep_moving = False
       

    # def record_depth(self):
    #     print("here")
    #     box = self.subject
    #     self.area = box.height*box.width
    #     if box is not None and (self.area < 0.8*se):
    #         self.keep_moving = True
    #         center_x = int(box.top_left_x + box.width/2)
    #         center_y = int(box.top_left_y + box.height/2)
    #         width_limit = int(box.width/10) 
    #         height_limit = int(box.height/10)
    #         depth = 0

    #         for i in range(0,10):
    #             x_rand = random.randint(center_x-width_limit,center_x+width_limit)
    #             y_rand = random.randint(center_y-height_limit,center_y+height_limit) 
    #             depth += (self.image[y_rand][x_rand])
    #         depth /= 10
    #         self.subject_depth = depth
    #         print(depth)
    #     else:
    #         self.keep_moving = False



    
if __name__=="__main__":
    rospy.init_node("bbox_filter")
    root = Isolate()
    while not rospy.is_shutdown():
        root.lets_move()
    rospy.spin()
