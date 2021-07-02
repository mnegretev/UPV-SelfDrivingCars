#!/usr/bin/env python

# rostopic pub -1 /AutoNOMOS_mini/manual_control/speed std_msgs/Int16 "data: 0"
# rosrun lane_detector simple_control.py _pbl_file:="/home/hector/source_code/upv-autonomous-car/catkin_ws/src/knowledge/pbl_files/evasion.pbl"
# rostopic pub -1 /AutoNOMOS_mini/manual_control/steering std_msgs/Int16 "data: 90"

import rospy
import cv2
import cv_bridge
import numpy
import math

from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from problog.program import PrologString
from problog.core import ProbLog
from problog import get_evaluatable
   
    
###############################################
##
##
def callback_rgb_raw(msg):
    global best_line
    last_rho   = best_line[0]
    last_theta = best_line[1]
    bridge = cv_bridge.CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    edges = cv2.Canny(img_bgr,100,200)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 90, None, 0, 0)
    if lines is None:
        return
    min_dist = 999999
    best_rho   = 0
    best_theta = 0
    for l in lines:
        rho, theta = l[0][0], l[0][1]
        if theta < 0:
            theta+=math.pi
        if abs(theta - last_theta) < min_dist:
            min_dist   = abs(theta - last_theta)
            best_rho   = rho
            best_theta = theta
    if min_dist > 0.2:
        return
    pt1 = (int(best_rho*math.cos(best_theta) + 1000*(-math.sin(best_theta))), int(best_rho*math.sin(best_theta) + 1000*(math.cos(best_theta))))
    pt2 = (int(best_rho*math.cos(best_theta) - 1000*(-math.sin(best_theta))), int(best_rho*math.sin(best_theta) - 1000*(math.cos(best_theta))))
    A = pt1[1] - pt2[1]
    B = pt2[0] - pt1[0]
    C = -A*pt1[0] - B*pt1[1]
    best_line = [best_rho, best_theta, A, B, C]
    cv2.line(img_bgr, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    cv2.imshow("Image BGR", img_bgr)
    cv2.waitKey(1)


###############################################
##
##
def callback_laser_scan(msg):

        global query_str
        global program_str  
        global answer_str
              
        #front = msg.ranges[324:360] + msg.ranges[0:36] # en contra de las manecillas del reloj 
	#print "Frontal readings: ",   front, "\n\n"
	#print "Numero de lecturas: ", len(front), "\n\n"
		
	lrL = min(msg.ranges[23:36])
	if math.isinf(lrL):
		lrL = 20 # default value
	
	temp_list = msg.ranges[0:23] + msg.ranges[337:360]	
	lrC = min(temp_list)
	if math.isinf(lrC):
		lrC = 20 # default value
		
	lrR = min(msg.ranges[325:337])
	if math.isinf(lrR):
		lrR = 20 # default value
			
        query_str = "query(action(" + str(lrL) + "," + str(lrC) + "," + str(lrR) + "," + " A))."
        print(query_str,  "\n")
        query_str = program_str + "\n\n" +  query_str
        query_str = PrologString(query_str)  
        answer_str =  str(get_evaluatable().create_from(query_str).evaluate())
	#print("answer_str: " + answer_str + "\n")
    
###############################################
##
##
def main():
  
    global query_str
    global program_str  
    global answer_str
    
    query_str = ""  
    program_str = ""
    answer_str = ""
    
    rospy.init_node("lane_finder")
    
    print("INITIALIZING TEST FOR PYSWIP")
    pbl_file = "/home/marco/PROYECTOS/UPV-SelfDrivingCars/catkin_ws/src/knowledge/pl_files/evasion.pbl"    
    if rospy.has_param("~pbl_file"):
        pbl_file = rospy.get_param("~pbl_file")
    if pbl_file == "":
        print("PbL file must be specified")
        return

    print("PL File: " + pbl_file)
    # Read the ProbLog program 
    with open(pbl_file, 'r') as file:
    	program_str = file.read()
    #print(program_str)
    

    print("INITIALIZING SIMPLE CONTROL BY MARCOSOFT...")
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)
    rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    pub_steering = rospy.Publisher("/AutoModelMini/manual_control/steering", Int16, queue_size=1)
    pub_speed    = rospy.Publisher("/AutoModelMini/manual_control/speed"   , Int16, queue_size=1)
    msg_steering = Int16()
    msg_speed    = Int16()
    #loop = rospy.Rate(30)   

    loop = rospy.Rate(10)
       
    global best_line
    best_line  = [0, 2.2863812,1,0,0]
    goal_angle = 2.2863812
    goal_dist  = 300.0
    
    action_str = ""
    while not rospy.is_shutdown():
        A = best_line[2]
        B = best_line[3]
        C = best_line[4]
        error_theta = goal_angle - best_line[1]
        dist = abs(A*320 + B*480 + C)/math.sqrt(A**2 + B**2)
        error_dist = goal_dist - dist 
        msg_steering.data = int(90 + error_dist/1.5 - 1.5*error_theta*180/math.pi)
        
        print("answer is: " + answer_str)
        if not answer_str == "":
                         
                max_prob = 0
                max_idx = 0  
		ipos = answer_str.find("go): ")
		if ipos >= 0:
		     offset = ipos + len("go): ")
		     subString = answer_str[offset:len(answer_str)]
		     fpos = subString.find(",")
		     if (fpos < 0): 
			 fpos = subString.find("}")
		     subString = answer_str[offset:offset + fpos]
		     #print("subString: ", subString, "Offset: ",  offset, "ipos: ", ipos, "fpos: ", fpos, "\n\n")
		     if float(subString) > max_prob:
		        max_prob = float(subString)
		        max_idx = 0 
			    
		ipos = answer_str.find("slowdown): ")            
		if ipos >= 0:
		     offset = ipos + len("slowdown): ")
		     subString = answer_str[offset:len(answer_str)]
		     fpos = subString.find(",")
		     if (fpos < 0): 
			 fpos = subString.find("}")
		     subString = answer_str[offset:offset + fpos]
		     if float(subString) > max_prob:
		        max_prob = float(subString)
		        max_idx = 1

		ipos = answer_str.find("stop): ")
		if ipos >= 0:
		     offset = ipos + len("stop): ")
		     subString = answer_str[offset:len(answer_str)]
		     fpos = subString.find(",")
		     if (fpos < 0): 
			 fpos = subString.find("}")
		     subString = answer_str[offset:offset + fpos]
		     if float(subString) > max_prob:
		        max_prob = float(subString)
		        max_idx = 2
		   
		ipos = answer_str.find("turnLeftSide): ")
		if ipos >= 0:
		     offset = ipos + len("turnLeftSide): ")
		     subString = answer_str[offset:len(answer_str)]
		     fpos = subString.find(",")
		     if (fpos < 0): 
			 fpos = subString.find("}")
		     subString = answer_str[offset:offset + fpos]
		     if float(subString) > max_prob:
		        max_prob = float(subString)
		        max_idx = 3
		     	     
		ipos = answer_str.find("turnRightSide): ")
		if ipos >= 0:
		     offset = ipos + len("turnRightSide): ")
		     subString = answer_str[offset:len(answer_str)]
		     fpos = subString.find(",")
		     if (fpos < 0): 
			 fpos = subString.find("}")
		     subString = answer_str[offset:offset + fpos]
		     if float(subString) > max_prob:
		        max_prob = float(subString)
		        max_idx = 4

                action_list = ['go', 'slowdown', 'stop', 'turnLeftSide', 'turnRightSide']		 
		car_speed = [-400, -200, 0, -200, -200]
		msg_speed.data = car_speed[max_idx]
                   
                if max_idx == 3:
                   msg_steering.data = +150
                if max_idx == 4:
                   msg_steering.data = -150

                print("Selected action: ", action_list[max_idx], "with speed: ", str(msg_speed.data), "with steering: ", str(msg_steering.data), "\n\n")   

        # print error_dist
        # print msg_steering.data
        pub_steering.publish(msg_steering)
        pub_speed.publish(msg_speed)
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
        
#                max_prob = max(probs)
#		max_idx = probs.index(max_prob)  
		 
#		car_speed = [-400, -200, 0, -200, -200]
#		msg_speed.data = car_speed[max_idx]

#                action_list = ['go', 'slowdown', 'stop', 'turnLeftSide', 'turnRightSide']                   
#                if max_idx == 3:
#                   msg_steering.data = -50
#                if max_idx == 4:
#                   msg_steering.data = 50

#                print("Selected action: ", action_list[max_idx], "with speed: ", str(msg_speed.data), "with steering: ", str(msg_steering.data), "\n\n")   
        
