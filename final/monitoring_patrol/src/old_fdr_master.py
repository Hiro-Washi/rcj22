#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String, Float64
import smach
import smach_ros
import roslib
from happymimi_msgs.srv import StrTrg
# MC
from geometry_msgs.msg import Twist
from happymimi_navigation.srv import NaviLocation
base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl
# RECOGNITION
from happymimi_recognition_msgs.srv import (RecognitionFind, 
                                            RecognitionFindRequest, 
                                            RecognitionLocalizeRequest)
reco_path = roslib.packages.get_pkg_dir('recognition_processing') + '/src/'
sys.path.insert(0, reco_path)
from recognition_tools import RecognitionTools
# VOICE
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from happymimi_voice_msgs.srv import *
import roslib.packages
#aram_wav = (roslib.packages.get_pkg_dir("happymimi_voice")
                                +"/../config/wave_data/aram.wav")
#tarminater_wav = (roslib.packages.get_pkg_dir("happymimi_voice")
                                +"/../config/wave_data/ga9du-ecghy2.wav")
from real_time_navi.srv import RealTimeNavi
from playsound import playsound
from send_gmail.srv import SendGmail

tts_sc = rospy.ServiceProxy('/tts', StrTrg)
rt = RecognitionTools()


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_finish'])
        self.navi_sc = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        #self.navi_sc= rospy.ServiceProxy("/apps/ad_navi_server", AdNaviSrv)
        self.bc = BaseControl()
        
    def execute(self, userdata):
        rospy.loginfo("Executing state: Start")
        tts_sc("Start patrolling")
        #self.navi_sc('start')
        self.bc(-45, 0.5)
        return 'start_finish'

class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',"found_no_one"],
                                input_keys=["search_count_in"])
        self.navi_sc = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.rtnavi_sc = rospy.ServiceProxy('/realtime_navi_server', RealTimeNavi)
        #self.navi_sc= rospy.ServiceProxy("/apps/ad_navi_server", AdNaviSrv)
        self.find_sc = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        #self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        self.bc = BaseControl()
 
    def execute(self, userdata):
        print("Executing state : SearchPerson")
        userdata.search_count_in =+ 1
        #navi_counter =+ 1
        self.head_pub.publish(30)
        rospy.sleep(1.5)
        self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
        self.head_pub.publish(0)
        rospy.sleep(1.0)
        # ??????????????????
        if self.find_result == True:
            print("found a person")
            # ????????????
            # request = RecognitionLocalizeRequest()
            # request.target_name = "person"
            # centroid = rt.localizeObject(request).point
            centroid = rt.localizeObject(RecognitionLocalizeRequest(target_name="person")).point
            person_height = centroid.z
            print(person_height)
            std_z = 0.4
            # Standing
            if person_height >= std_z:
                self.head_pub.publish(0)
                tts_srv("Hi!")
                target_name = 'standing_person'
                print('found a standing person')
                tts_srv("HHow are you???")
                return 'found_standing'
            # Lying
            else:
                tts_srv("Found a lying person")
                target_name = 'lying_person'
                # ?????????????????????
                self.rtnavi_sc('add', "lying_person")
                return 'found_lying'
        # ?????????????????????
        elif self.find_result == False:
            print("found a person.")
            #tts_srv("")
            if userdata.search_count_in == 1:
                #tts_srv("moving to another point")
                self.navi_sc('search_2')
                rospy.sleep(0.5)
                rospy.loginfo('finish moving to another place')
                return 'not_found_one'
            elif navi_counter > 1:
                print("found no person again")
                tts_srv("Found no person again. He is out of this house now")
                return 'not_found_two'

class TalkAndAlert(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['to_call','to_exit'])
        #self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        #self.rt = RecognitionTools()
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.mail_srv = rospy.ServiceProxy('/send_gmail_server', SendGmail)
    def execute(self, userdata):
        print("Executing state : TalkAndAlert")
        for i in range(3):
            tts_srv("Are you sleeping")
            #yes_no_result = self.yes_no_srv().result#####
            self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
            # ?????????????????????????????????
            if self.find_result == False:
                print('The person is away here')
                tts_srv("You are out of my eyes. You probably woke up")
                return 'to_exit'
            #????????????????????????????
            elif self.find_result == True:
                pass
            request = RecognitionLocalizeRequest()
            request.target_name = "person"
            centroid = rt.localizeObject(request).point
            person_height = centroid.z
            print(person_height)
            standard_z = 0.4
            #if person_height > standard_z or yes_no_result == True or yes_no_result == False:  #???voice
            #????????????????????????????????????????????????
            if person_height > standard_z :
                self.head_pub.publish(0)
                print("Confirm that you are awake")
                #tts_srv("How are you?")
                #tts_srv("I'm sorry to prevent you from sleeping though, I think it's better to sleep on your facking bed")
                return 'to_exit'
                break
            else:
                pass
        #??????????????????????????????????????????
        else:
            for i in range(3):
                playsound(happymimi_voice_path)
                print('send mail for help')
                self.mail_srv('kit.robocup.home@gmail.com',
                              'gewretvedgpzlobj',
                             ['c1100781@planet.kanazawa-it.ac.jp',
                              'c1115332@planet.kanazawa-it.ac.jp'],
                              '???KitHappyRobot?????????????????????????????????',
                              '??????????????????????????????????????????????????????' + '?????????????????????????????????????????????',
                             )
                tts_srv("A person is lying down and lose conciousness")
                request = RecognitionLocalizeRequest()
                request.target_name = "person"
                centroid = rt.localizeObject(RecognitionLocalizeRequest(target_name="person")).point
                person_height = centroid.z
                print(person_height)
                standard_z = 0.4
                
                if person_height > standard_z:
                    self.head_pub.publish(0)
                    print("Confirm that a person stand")
                    tts_srv("Waht's up. Are you OK?")
                    tts_srv("take a nice nap")
                    return 'to_exit'
                    break
                else:
                    pass
            else:
                return 'to_call'

class Call(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['call_finish'])
        self.navi_sc = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.rtnavi_sc = rospy.ServiceProxy('realtime_navi_server', RealtimeNavi)
        #self.navi_sc = rospy.ServiceProxy("/apps/ad_navi_server", AdNaviSrv)
        self.find_sc = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()
    
    def execute(self, userdata):
        print("Executing state : Call")
        tts_srv("I'll call for help")
        self.navi_sc("outside_1")
        for i in range(2):
            #playsound(aram)
            tts_srv("Need Help! A person is losing consciousness. Please call an ambulance")
            self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
            # Detect human
            if self.find_result == True:
                tts_srv("I'll guide you, please follow me.")
                self.head_pub.publish(0)
                self.rtnavi_sc('navigation',"lying_person")
                #tts_srv("I guided you to the unconscious person")
                return 'call_finish'
                break
            elif self.find_result == False:
                if i == 0:
                    self.bc.rotateAngle(90, 0.3)
                    #rospy.sleep(2.0)
                elif i == 1:
                    self.bc.rotateAngle(-180,0.5)
                elif i == 2:
                    self.bc.rotateAngle(90,0.4)
                else:
                    break
        return 'call_finish'

class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
    def execute(self, userdata):
        print("Start going to operator")
        tts_srv("I'll be back")
        start_time = time.time()
        stop_time = 7
        while (time.time() - start_time) <= stop_time:
            playsound(sec_happymimi_voice_path)

        self.navi_srv("start_point")
        print("finish confirming of human life safety")
        tts_srv("finish confirming")
        return 'all_finish'

if __name__ == '__main__':
    rospy.init_node('falling_down_resc')
    rospy.loginfo('Start informing people lying down')
    top = smach.StateMachine(outcomes = ['finish_sm_top'])
    top.userdata.search_count = 0
    with top:
        smach.StateMachine.add(
                'Start',
                Start(),
                transitions = {'start_finish':'SearchPerson'})
        smach.StateMachine.add(
                'SearchPerson',
                SearchPerson(),
                transitions = {'found_lying':'TalkAndAlert',
                               'found_standing':'Exit',
                               'found_no_one':'Exit'},
                remapping   = {"search_count_in":"search_count"})
        smach.StateMachine.add(
                'TalkAndAlert',
                TalkAndAlert(),
                transitions = {'to_call':'Call',
                               'to_exit':'Exit'})
        smach.StateMachine.add(
                'Call',
                Call(),
                transitions = {'call_finish':'Exit'})
        smach.StateMachine.add(
                'Exit',
                Exit(),
                transitions = {'all_finish':'finish_sm_top'})
        
    outcome = top.execute()
