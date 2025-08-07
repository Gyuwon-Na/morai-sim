#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from warped_img import Lane_sub

if __name__ == "__main__":
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
