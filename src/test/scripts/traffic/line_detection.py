#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.steer_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()

    def cam_CB(self,msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(img_hsv)
        y,x = img.shape[0:2]
        # cv2.imshow("h",h)
        # cv2.imshow("s",s)
        # cv2.imshow("v",v)

        yellow_lower = np.array([15,128,0])  
        yellow_upper = np.array([45,255,255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0,0,192])  
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_range = cv2.bitwise_and(img,img,mask = combined_range)
        
        src_point1 = [0,420]
        src_point2 = [275,260]
        src_point3 = [x-275,260]
        src_point4 = [x,420]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_range, matrix, [x,y])

        grayed_img = cv2.cvtColor(warped_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img>50]=255
        histogram_x = np.sum(bin_img, axis=0)
        histogram_y = np.sum(bin_img, axis=1)

        left_hist = histogram_x[0:x//2]
        right_hist = histogram_x[x//2:]
        left_indices = np.where(left_hist>20)[0]
        right_indices = np.where(right_hist>20)[0]+320

        cross_indices = np.where(histogram_y > 400)[0]

        try:
            cross_threshold = 25
            cross_diff = cross_indices[-1] - cross_indices[0]
            if cross_threshold < cross_diff:
                self.cross_flag = True
                #cv2.line(warped_img,[0,cross_indices[0]],[x,cross_indices[-1]],[0,255,0],3)
            else:
                self.cross_flag = False
        except:
                self.cross_flag = False

        indices = np.where(histogram_x>20)[0]

        try:
            if len(left_indices) != 0 and len(right_indices) == 0:
                center_idx = (left_indices[0]+right_indices[-1])//2
            elif len(left_indices) == 0 and len(right_indices) != 0:
                center_idx = (right_indices[0] + left_indices[-1])//2
            else:
                center_idx = (indices[0] + indices[-1])//2
        except:
            center_idx = x//2
        
        lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, None, 20, 2)
        try:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(warped_img, (x1,y1), (x2,y2), (0,255,0), 1)
        except:
            pass
        standard_line = x//2
        degree_per_pixel = 1/x
        steer = (center_idx - standard_line) * degree_per_pixel
        steer += 0.5

        self.steer_msg.data = steer
        self.speed_msg.data = 1000
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

        cv2.imshow("img",img)
        cv2.imshow("warped_img",warped_img)
        cv2.imshow("bin_img",bin_img)
        
        cv2.waitKey(1)

def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()