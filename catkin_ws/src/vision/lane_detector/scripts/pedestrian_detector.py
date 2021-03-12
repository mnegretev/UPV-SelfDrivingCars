#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy
import math

import glob
import random

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16




def callback_rgb_raw(msg):

    net = cv2.dnn.readNet("/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/pedestrian_detector.weights", "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov3_configuration.cfg")
    # Name custom object
    classes = ["person"]

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = numpy.random.uniform(0, 255, size=(len(classes), 3))

    bridge = cv_bridge.CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    img = cv2.resize(img_bgr, None, fx=1, fy=1)
    height, width, channels = img.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = numpy.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.3:
                # Object detected
                print(class_id)
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    print(indexes)
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            color = colors[class_ids[i]]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label, (x, y + 30), font, 3, color, 2)


    cv2.imshow("Image", img)
    cv2.waitKey(50)


def main():
    # Load Yolo

    print("Pedestrian YOLOv3")

    rospy.init_node("pedestrian")
    #rospy.Subscriber("/scan", LaserScan, callback_laser_scan)

    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)

    loop = rospy.Rate(30)




    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
