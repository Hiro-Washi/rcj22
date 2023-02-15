#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# just a location_name requesst is preferred
import rospy
import rosparam
import actionlib
import dynamic_reconfigure.client

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import sys, roslib
mimi_navi_path = roslib.packages.get_pkg_dir("happymimi_navigation")
sys.path.insert(0, mimi_navi_path)
#sys.path.insert(0, "${HOME}/test_ws/src/dev_noe/task/rcj22/final")
#from happymimi_navigation.srv import NaviCoord, NaviCoordRequest
from monitoring_patrol.srv import AdNaviSrv, AdNaviSrvResponse


class AdNaviServer():
  def __init__(self):
    # TOPIC
    # Publisher
    self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
    
    # SERVICE
    rospy.loginfo("ready to **ad_navi_server**")
    self.an_ss = rospy.Service('/mp/ad_navi_srv', AdNaviSrv, self.execute)
    self.dwa_ac = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
    self.clear_costmap_sc = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    # ACTION
    self.mb_ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    # PARAM/DICT
    #self.navi_params_dict = rospy.get_param("/ad_navi_param")
    self.navi_def_params = rospy.get_param("/ad_navi_param/FastLoose")
    self.navi_fl_params = rospy.get_param("/ad_navi_param/Default")
    self.location_dict = rospy.get_param("/location_dict")
    self.target_coord_list = [] #!!

  def setParam(self, option = "default"): #!!
    if option == "fast_loose":
      self.dwa_ac.update_configuration(self.navi_fl_params)
    elif option == "default":
      self.dwa_ac.update_configuration(self.navi_def_params)
    rospy.sleep(0.5)
    
  def name2Coord(self, location):
    if location in self.location_dict:
      self.target_coord = self.location_dict[location]
      rospy.loginfo("AdNaviServer: target location ->" +location)
      return True
    else: 
      rospy.logerr("AdNaviServer: <"+ location +"> doesn't exist.")
      return False
  
  def sendGoal(self, coord_list):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coord_list[0]
    goal.target_pose.pose.position.y = coord_list[1]
    goal.target_pose.pose.orientation.z = coord_list[2]
    goal.target_pose.pose.orientation.w = coord_list[3]
    # Look forward
    self.head_pub.publish(0)
    rospy.sleep(0.5)
    # Clear costmap
    rospy.loginfo("AdNaviServer: Clearing costmap...")
    rospy.wait_for_service('move_base/clear_costmaps')
    self.clear_costmap_sc()
    # Request to MoveBaseActionServer
    self.mb_ac.wait_for_server()
    self.mb_ac.send_goal(goal)
    #self.mb_ac.wait_for_result()
    navi_state = self.mb_ac.get_state()
    while not rospy.is_shutdown():
      navi_state = self.mb_ac.get_state()
      if navi_state == 3:
        rospy.loginfo('AdNaviServer: Navigation succeeded!!')
        return AdNaviSrvResponse(result = True)
      elif navi_state == 4:
        rospy.loginfo('AdNaviServer: Navigation Failed...')
        return AdNaviSrvResponse(result = False)
      else:
        rospy.loginfo("AdNaviServer: Navigation didn't go well (navi_state -> "+ navi_state)
        return AdNaviSrvResponse(result = False)
  
  def execute(self, req):
    rospy.loginfo("AdNaviServer: Execute")
    
    # Set the target coord_list
    if req.location_name == "":
      self.target_coord_list = req.coordinate_list
    else: # in case of recieving a location name
      # Judge name2Coord result
      if self.name2Coord(req.location_name): pass
      else: 
        rospy.loginfo("AdNaviServer: Failed to gen-coord from the location")
        return AdNaviSrvResponse(result = False)
    
    # Set the navigation parameter according to requested option
    self.setParam(option = req.option)
    # Send goal
    self.sendGoal(self.target_coord_list)
    # Fix navi_params
    self.setParam(option = "default")
      
      
def main():
  rospy.init_node('ad_navi_server')
  try:
    ans = AdNaviServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()