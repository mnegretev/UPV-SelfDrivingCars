#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

def calculate_inclination_angle_list(lines):
    try:
        if len(lines)==0:
            print("No lines found")
            pass
        else:
            slope_list = []

            if lines is not None:
                #print("ASDF")
                i=0
                for linea in lines:
                    for x1,y1,x2,y2 in lines[i]:
                        slope = (float(y2)-float(y1)) / (float(x2)-float(x1))
                        slope = (math.atan(slope)*180)/math.pi
                        slope_list.append(slope)

                    i=i+1
                i=0
        return(slope_list)
    except:
        pass

def region_of_interest(img_edges):
    height = img_edges.shape[0]
    width = img_edges.shape[1]
    mask = numpy.zeros_like(img_edges)
    px1, py1 =-500, height,
    px2, py2 =320, 200
    px3, py3 = 670, height
    #triangulo
    triangle = numpy.array([[(px1,py1),(px2, py2),(px3, py3)]],numpy.int32)
    cv2.fillPoly(mask,triangle,255)
    img_masked = cv2.bitwise_and(img_edges, mask)
    #cv2.line(img_masked,(px1,py1),(px2,py2),(255,0,0),2)
    #cv2.line(img_masked,(px2,py2),(px3,py3),(255,0,0),2)
    #cv2.line(img_masked,(px3,py3),(px1,py1),(255,0,0),2)
    return img_masked


def callback_rgb_raw(msg):
    vel = Int16()
    direction = Int16()

    bridge = cv_bridge.CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    lower_white = numpy.array([100, 100, 100])
    upper_white = numpy.array([255, 255, 255])
    binary_image = cv2.inRange(img_bgr, lower_white, upper_white)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))
    filtered_image =  cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(filtered_image,100,200)
    #cropped_image = region_of_interest(Edges_image)
    lines = cv2.HoughLinesP(edges, rho=1, theta = numpy.pi/180, threshold =60, lines=None,minLineLength=125,maxLineGap=150)
    #lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 150, None, 0, 0)

    line_color=[255,0,0]
    line_thickness = 3
    dot_color = [0,0,255]
    dot_size=9
    if lines is not None:
        i=0
        vlines = []
        for linea in lines:
            for x1,y1,x2,y2 in lines[i]:
                deltay = abs(y2 - y1)
                deltax = abs(x2 - x1)
                slope = (float(y2)-float(y1)) / (float(x2)-float(x1))
                angle = (math.atan(slope)*180)/math.pi

                #print(angle)
                if (angle < -5 or angle > 5):
                    vlines.append(linea)
                    cv2.line(img_bgr,(x1,y1),(x2,y2),line_color,line_thickness)
                #    cv2.circle(img_bgr,(x1,y1),0,dot_color,dot_size)
                #    cv2.circle(img_bgr,(x2,y2),0,dot_color,dot_size)
            i=i+1
        i=0
    cv2.imshow("Cany Borders", edges)
    cv2.imshow("Image BGR", img_bgr)
    cv2.waitKey(1)

    angle_list = calculate_inclination_angle_list(vlines)
    #print(len(angle_list))
    if isinstance(angle_list,list) == True:

        upper_angle_list = []
        for angle in angle_list:
            if angle < 0:
                upper_angle_list.append(angle)
        upper_angle_list.sort()
        print(upper_angle_list[0])
        if abs(upper_angle_list[0]) > 36.5:
            print("The car is Straight")
            vel.data=-500
            direction.data=90
        else:
            print("The car is NOT Straight")
            angle_error= 39 - abs(upper_angle_list[0])
            vel.data=-350
            direction.data = 90 - (int(angle_error)*90)/39
        pub_speed.publish(vel)
        pub_steering.publish(direction)
        #print(upper_angle_list[0])


def main():
    global pub_steering, pub_speed
    print("Lane follower v0")
    rospy.init_node("lane_finder1")
    #rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    pub_steering = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size=1)
    pub_speed    = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed"   , Int16, queue_size=1)

    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)

    loop = rospy.Rate(30)




    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
