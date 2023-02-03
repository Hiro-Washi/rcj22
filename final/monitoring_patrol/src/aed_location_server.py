#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Use def location value(Yumeko test spase: 36.53247090438674, 136.63104443023042)
#  cause the accuracy of Geojs api is not good. 

# Geolocation accuracy

# Gmap point: 
#  - LC: 36.53163897927895, 136.6271756167569
#  - Yumeko test spase: 36.53247090438674, 136.63104443023042

# API point : latitude":"36.5968 longitude":"136.5998"

from yaml import load
import requests
import rospy, rosparam
from std_msgs.msg import String, Float64

import sys;
sys.path.insert(0, '/home/hiroto/test_ws/src/dev_noe/task/rcj22/final')
from monitoring_patrol.srv import AedlocationInfo, AedlocationInfoResponse

class AedLocationServer():
  def __init__(self):
    rospy.loginfo('Start "AED locatioin server"')
    rospy.Service('acd_location_server', AedlocationInfo, self.aedLocationCB)
    self.yumeko_test_space_loc = [36.53247090438674, 136.63104443023042]
  
  # return nearest AED location. Goejs 
  def aedLocatinCB(self, request):
    if True:
      return 

  
def main():
  rospy.init_node('aed_location_server', anonymous=True)
  try:
    ALS = AedLocationServer()
    rospy.spin()
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
    main()