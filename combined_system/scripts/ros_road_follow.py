#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import os
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from jetcam.utils import bgr8_to_jpeg
from jetcam.csi_camera import CSICamera
from torch2trt import TRTModule
import torch.nn.functional as F
from collections import deque
import PIL.Image
import torchvision.transforms as transforms
from visualization_msgs.msg import Marker

class RoadFollow:
    def __init__(self):
        rospy.init_node("road_follower", anonymous=True)
        time_start = rospy.get_time()
        self.is_enabled = True
        self.count = 0
        self.twist = Twist()
        self.prev_conf = deque(maxlen=10)

        self.camera = CSICamera(
            width=224,
            height=224,
            capture_fps=10,
            capture_device=0
        )
        

        road_model_path = rospy.get_param('~road_model_path', os.path.join(os.path.dirname(__file__), "models", "updated_model_trt.pth"))
        detect_model_path = rospy.get_param('~detect_model_path', os.path.join(os.path.dirname(__file__), "models", "best_model_trt.pth"))
        self.enable_vis = rospy.get_param("enable_vis", "false")

        self.road_model_trt = TRTModule()
        self.road_model_trt.load_state_dict(torch.load(road_model_path))
        self.road_model_trt.cuda().half().eval()

        self.detect_model_trt = TRTModule()
        self.detect_model_trt.load_state_dict(torch.load(detect_model_path))
        self.detect_model_trt.cuda().half().eval()
        
        self.steering_pub = rospy.Publisher("/steering_value", Float32, queue_size=10)
        self.enable_sub = rospy.Subscriber("/ml_enable", Bool, self.enable_callback)
        self.confidence_check = rospy.Publisher("/conf_check", Bool, queue_size=10)
        self.marker_pub = rospy.Publisher("/rf_vis", Marker, queue_size=10)
        
        init_time = rospy.get_time() - time_start
        rospy.loginfo("Road follower node initialized")
        rospy.loginfo(f"Init time: {init_time}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.count += 1
            new_image = self.camera.read()
            image = preprocess(new_image).half().cuda()
            if self.count % 2 == 0 or self.count == 1:
                road_conf = self.road_confidence(image)
                self.prev_conf.append(road_conf)
            if self.is_enabled:
                if np.mean(self.prev_conf) > 0.8 and len(self.prev_conf) > 5:
                    rospy.loginfo("Road confidence low. Stopping Road following")
                    self.confidence_check.publish(Bool(False))
                with torch.no_grad():
                    output = self.road_model_trt(image).detach().cpu().numpy().flatten()
                steering = float(output[0])
                self.steering_pub.publish(steering)
                rate.sleep()
            else:
                if road_conf < 0.2:
                    self.confidence_check.publish(Bool(True))
                else:
                    rate.sleep()
            if self.enable_vis == "true":
                visualise(steering, road_conf)

    def road_confidence(self, image): 
        y = self.detect_model_trt(image)
        y = F.softmax(y, dim=1)
        prob_not_road = float(y.flatten()[0])
        return prob_not_road

    def enable_callback(self, msg):
        self.is_enabled = msg.data
        
    def visualise(self, steering_value, road_conf):
        steer_marker = Marker()
        steer_marker.header.frame_id = "base_link"
        steer_marker.header.stamp = rospy.Time.now()
        steer_marker.type = Marker.ARROW
        steer_marker.action = Marker.ADD
        steer_marker.pose.position.x = 0.5
        steer_marker.pose.position.y = 0
        steer_marker.pose.position.z = 0.2
        steer_marker.pose.orientation.w = 1.0
        steer_marker.scale.x = 0.3  
        steer_marker.scale.y = 0.05  
        steer_marker.scale.z = 0.05  
        steer_marker.color.r = 0.0
        steer_marker.color.g = 1.0  
        steer_marker.color.b = 0.0
        steer_marker.color.a = 1.0  
        steer_marker.pose.orientation.z = np.sin(steering_value / 2.0)
        steer_marker.pose.orientation.w = np.cos(steering_value / 2.0)
        self.marker_pub.publish(steer_marker)

        confidence_marker = Marker()
        confidence_marker.header.frame_id = "base_link"
        confidence_marker.header.stamp = rospy.Time.now()
        confidence_marker.type = Marker.SPHERE
        confidence_marker.action = Marker.ADD
        confidence_marker.pose.position.x = 0.5
        confidence_marker.pose.position.y = 0
        confidence_marker.pose.position.z = 0.5
        confidence_marker.pose.orientation.w = 1.0
        confidence_marker.scale.x = 0.1
        confidence_marker.scale.y = 0.1
        confidence_marker.scale.z = 0.1
        #if confident it is road then green
        if road_conf < 0.2:
            confidence_marker.color.r = 0.0
            confidence_marker.color.g = 1.0
            confidence_marker.color.b = 0.0
        #if not then red
        else:
            confidence_marker.color.r = 1.0
            confidence_marker.color.g = 0.0 
            confidence_marker.color.b = 0.0

        confidence_marker.color.a = 1.0
        self.marker_pub.publish(confidence_marker)

def preprocess(image):
    mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
    std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
    device = torch.device('cuda')
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]


            

if __name__ == '__main__':
    rf = RoadFollow()
    rf.run()
