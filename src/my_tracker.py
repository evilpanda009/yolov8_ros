#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from yolov8_ros.msg import Box,Boxes
import time
import random
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

class Isolate():
    def __init__(self):
        self.box_sub = rospy.Subscriber('/yolov8_boxes', Boxes, self.box_callback)
        # self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_width = 0
        self.image_height = 0
        self.boxes = Boxes()
        self.depths = []
        self.subject_depth = -1
        self.cx_old = 0
        self.center = 320
        self.linear_vel = 0.2
        self.angular_vel = 0.05
        self.bb = []
        self.subject = Box()
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
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

    def rotate_with_linear(self, rotation, forward):
        msg = Twist()
        msg.linear.x = self.linear_vel if forward else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = rotation* self.angular_vel
        self.pub_vel.publish(msg)

    def get_cx_cy(self):
        if len(self.bb)!=0:
            xmin = self.subject.top_left_x
            ymin = self.subject.top_left_y
            xmax = xmin + self.subject.width
            ymax = ymin + self.subject.height
            self.cx_old = int((xmin + xmax)/2)

    def lets_move(self):
        if len(self.bb)!=0:    
            self.get_cx_cy()
            error = float(self.center - self.cx_old)
            while(abs(error) > 30 and not rospy.is_shutdown()):
                    self.get_cx_cy()                  
                    error = float(self.center - self.cx_old)
                    rotation = 1 if (error>0) else -1 # decides direction of rotation
                    rotation_time = error/100
                    self.rotate_with_linear(rotation, True)
                    rospy.sleep(0.05)
            self.stop()
            while(self.subject_depth > 1 and not rospy.is_shutdown()):
                self.move_forward()
            self.stop()


    def box_callback(self,msg):
        self.boxes = msg.Boxes if len(msg.Boxes)>0 else []
        self.bb = self.boxes
        self.subject = self.bb[0] if len(self.bb)>0 else None
        # Write service here to select any ID instead of 0

    def depth_callback(self, msg):
        bridge = CvBridge()
        self.depths = bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.image_height = msg.height
        self.image_width = msg.width
        self.center = self.image_width/2

    def record_depth(self):
        min_depth = 10000000
        self.temp_depth = -1
        box = self.subject
        if box is not None:
            center_x = int(box.top_left_x + box.width/2)
            center_y = int(box.top_left_y + box.height/2)
            width_limit = int(box.width/10) 
            height_limit = int(box.height/10)
            depth = 0

            for i in range(0,10):
                x_rand = random.randint(center_x-width_limit,center_x+width_limit)
                y_rand = random.randint(center_y-height_limit,center_y+height_limit) 
                depth += (self.depths[y_rand][x_rand])
            depth /= 10
            self.subject_depth = depth


    
if __name__=="__main__":
    rospy.init_node("bbox_filter")
    root = Isolate()
    while(True and not rospy.is_shutdown()):
        root.lets_move()
    rospy.spin()
