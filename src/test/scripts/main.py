#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from camera import WarpedImg

if __name__ == "__main__":
    try:
        lane_sub = WarpedImg.Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
