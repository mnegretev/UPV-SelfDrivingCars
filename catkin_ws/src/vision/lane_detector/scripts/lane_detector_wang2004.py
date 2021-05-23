#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

def display_lines(img_bgr, average_lines,d):
    img_lines=numpy.zeros((img_bgr.shape[0],img_bgr.shape[1],3),dtype=numpy.uint8)
    line_color=[255,0,0]
    line_thickness = 3
    dot_color = [0,0,255]
    dot_size=9
    #i = 0
    try:
        if average_lines is not None:
            for i in range(0, len(average_lines)):
                l = average_lines[i][0]
                slope = (float(l[3]+d)-float(l[1]+d)) / (float(l[2])-float(l[0]))
                if ((slope>0.3 and slope>0)or(slope<-0.3 and slope <0)):
                    #vlines.append(linea)
                    #l[1]= l[3]-d
                    #l[0]= l[2]-d
                    cv2.line(img_bgr, (l[0], (l[1])+d), (l[2], (l[3]+d)), (0,0,255), 3)
        return img_lines
    except:
        print("something went wrong")

def add_weighted(img_bgr, img_lines):
    try:
        return cv2.addWeighted(src1=img_bgr, alpha=0.8, src2=img_lines, beta=1.0, gamma =0.0)
    except:
        pass

def callback_rgb_raw(msg):
    bridge = cv_bridge.CvBridge()
    bgr_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    original_image= bgr_image.copy()
    lower_white = numpy.array([100, 100, 100])
    upper_white = numpy.array([255, 255, 255])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))
    #original_image = cv2.imread("/home/pp/Servicio_social/Yolov3-detector-Victor_Cruz/Yolov3-detector-20210205T234525Z-001/Yolov3-detector/speed-limit-detector/images/v-003.png")
    #original_image = cv2.imread("/home/pp/Servicio_social/Yolov3-detector-Victor_Cruz/prueba1.jpeg")
    #original_HSV = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    #binary_image = cv2.inRange(original_image, lower_white, upper_white)
    #filtered_image = cv2.morphologyEx(binary_image,cv2.MORPH_OPEN, kernel)
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))
    #filtered_image =  cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

    imwidth = original_image.shape[1]
    imheight= original_image.shape[0]
    #print(imwidth)
    #M = imwidth//5
    #N = imheight//5

    #for y in range(0,imheight,N):
        #y1 = y + N
    #tile1 = original_image[0:imwidth,0:int(imheight*0.1)]
    d0 = 0
    d1 = int(imheight*0.1)
    d2 = d1 + int(imheight*0.15)
    d3 = d2 + int(imheight*0.2)
    d4 = d3 + int(imheight*0.25)

    #distancias =[d0,d1,d2,d3,d4]
    dist=[]
    dist.append(d0)
    dist.append(d1)
    dist.append(d2)
    dist.append(d3)
    dist.append(d4)

    sec1 = original_image[d0:d1,0:imwidth]
    sec2 = original_image[d1:d2,0:imwidth]
    sec3 = original_image[d2:d3,0:imwidth]
    sec4 = original_image[d3:d4,0:imwidth]
    sec5 = original_image[d4:imheight,0:imwidth]

    imagenes = []
    imagenes.append(sec1)
    imagenes.append(sec2)
    imagenes.append(sec3)
    imagenes.append(sec4)
    imagenes.append(sec5)

    def add_weighted(img_bgr, img_lines):
        try:
            return cv2.addWeighted(src1=img_bgr, alpha=0.8, src2=img_lines, beta=1.0, gamma =0.0)
        except:
            pass

    lower_white = numpy.array([100, 100, 100])
    upper_white = numpy.array([255, 255, 255])

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))

    i=0
    #for imgs in imagenes:
    #    binary_image = cv2.inRange(imagenes[i], lower_white, upper_white)
    #    filtered_image =  cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    #    Edges_image = cv2.Canny(filtered_image,100,200)
    #    only_lines = cv2.HoughLinesP(Edges_image, rho=1, theta = numpy.pi/180, threshold =60, lines=None,minLineLength=125,maxLineGap=150)
    #    img_lines= display_lines(original_image, only_lines,dist[i])
        #cv2.imshow("Edges"+str(i),Edges_image)
    #    img_overlayed = add_weighted(original_image, img_lines)
    #    i=i+1
    ##Section 1
    binary_image1 = cv2.inRange(sec1, lower_white, upper_white)
    #filtered_image1 =  cv2.morphologyEx(binary_image1, cv2.MORPH_OPEN, kernel)
    #Edges_image1 = cv2.Canny(filtered_image1,100,200)
    Edges_image1 = cv2.Canny(binary_image1,100,200)
    only_lines1 = cv2.HoughLinesP(Edges_image1, rho=1, theta = numpy.pi/180, threshold =20, lines=None,minLineLength=20,maxLineGap=50)
    img_lines1= display_lines(original_image, only_lines1,d0)
    img_overlayed = add_weighted(original_image, img_lines1)
    ##Section 2
    binary_image2 = cv2.inRange(sec2, lower_white, upper_white)
    filtered_image2 =  cv2.morphologyEx(binary_image2, cv2.MORPH_OPEN, kernel)
    Edges_image2 = cv2.Canny(filtered_image2,100,200)
    only_lines2 = cv2.HoughLinesP(Edges_image2, rho=1, theta = numpy.pi/180, threshold =20, lines=None,minLineLength=20,maxLineGap=90)
    img_lines2= display_lines(original_image, only_lines2,d1)
    img_overlayed = add_weighted(original_image, img_lines2)
    ##Section 3
    binary_image3 = cv2.inRange(sec3, lower_white, upper_white)
    filtered_image3 =  cv2.morphologyEx(binary_image3, cv2.MORPH_OPEN, kernel)
    Edges_image3 = cv2.Canny(filtered_image3,100,200)
    only_lines3 = cv2.HoughLinesP(Edges_image3, rho=1, theta = numpy.pi/180, threshold =30, lines=None,minLineLength=25,maxLineGap=150)
    img_lines3= display_lines(original_image, only_lines3,d2)
    img_overlayed = add_weighted(original_image, img_lines3)
    ##Section 4
    binary_image4 = cv2.inRange(sec4, lower_white, upper_white)
    filtered_image4 =  cv2.morphologyEx(binary_image4, cv2.MORPH_OPEN, kernel)
    Edges_image4 = cv2.Canny(filtered_image4,100,200)
    only_lines4 = cv2.HoughLinesP(Edges_image4, rho=1, theta = numpy.pi/180, threshold =50, lines=None,minLineLength=30,maxLineGap=150)
    img_lines4= display_lines(original_image, only_lines4,d3)
    img_overlayed = add_weighted(original_image, img_lines4)
    ##Section 5
    binary_image5 = cv2.inRange(sec5, lower_white, upper_white)
    filtered_image5 =  cv2.morphologyEx(binary_image5, cv2.MORPH_OPEN, kernel)
    Edges_image5 = cv2.Canny(filtered_image5,100,200)
    only_lines5 = cv2.HoughLinesP(Edges_image5, rho=1, theta = numpy.pi/180, threshold =50, lines=None,minLineLength=50,maxLineGap=50)
    img_lines5= display_lines(original_image, only_lines5,d4)
    img_overlayed = add_weighted(original_image, img_lines5)


    cv2.imshow("sec1",sec1)
    cv2.imshow("sec2",sec2)
    cv2.imshow("sec3",sec3)
    cv2.imshow("sec4",sec4)
    cv2.imshow("sec5",sec5)
    #cv2.rectangle(original_image,(0,0),(imwidth,d0),(0,255,0))
    #cv2.rectangle(original_image,(0,d0),(imwidth,d1),(0,255,0))
    #cv2.rectangle(original_image,(0,d1),(imwidth,d2),(0,255,0))
    #cv2.rectangle(original_image,(0,d2),(imwidth,d3),(0,255,0))
    #cv2.rectangle(original_image,(0,d3),(imwidth,imheight),(0,255,0))

    #img_lines= display_lines(original_image, only_lines)


    #cv2.imshow("edges",Edges_image3)
    cv2.imshow("original", original_image)
    #cv2.imshow("real", bgr_image)
    cv2.waitKey(1)

def callback_laser_scan(msg):
    print("Laser scan received with " + str(len(msg.ranges)) + " received")

def main():
    print("B-snake Nepptuno")
    rospy.init_node("lane_finder")
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)
    #rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
