#!/usr/bin/env python
import cv2
import cv_bridge
import numpy as np
import glob
import rospy
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image

pub_cmd_vel = None
pub_detection= None


def callback_rgb_raw(msg):
    bridge = cv_bridge.CvBridge()
    bgr_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    img= bgr_image.copy()

    height, width, channels = img.shape
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []
    centros=[]

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                centros.append([center_x,center_y])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    font = cv2.FONT_HERSHEY_PLAIN
    for ii in range(len(boxes)):
        if ii in indexes:
            x, y, w, h = boxes[ii]
            m,n = centros[ii]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0,255,0), 2)
            cv2.putText(img, classes[class_ids[ii]], (x, y + 30), font, 2, (0,255,0), 2)

            #else:
                #detection= "no se encontro"
                #pub_detection.publish(detection)


    cv2.imshow('yolo',img)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.destroyAllWindows()
        return


def main():
    global classes

    print("Yolo sin pkg - Nepptuno")
    print(cv2.__version__)
    rospy.init_node("YOLO")

    #path1 = os.path.abspath(__file__)[:-10] + "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.weights"
    #path2 = os.path.abspath(__file__)[:-10] + "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.cfg"
    #Path tiny-yolov2:
    path1="/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.weights"
    path2="/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.cfg"
    #Pth tiny:
    #path1="/home/pp/UPV-SelfDrivingCars/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/tiny.weights"
    #path2="/home/pp/UPV-SelfDrivingCars/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg/tiny.cfg"
    global net
    net = cv2.dnn.readNet(path1, path2)
    #net = cv2.dnn.readNet("/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.weights", "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/yolov2-tiny.cfg")
    #classes_yolo2 = ["aeroplane","bicycle","bird","boat","bottle","bus","car","cat","chair","cow","diningtable","dog","horse","motorbike",
              # "person","pottedplant","sheep","sofa","train","tvmonitor"]

    classes = ["person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat","traffic light","fire hydrant",
                "stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
                "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
                "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon",
                "bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa",
                "pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone",
                "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
                "toothbrush"]

    global layer_names
    layer_names = net.getLayerNames()
    global output_layers
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    #print(np.size(output_layers))
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)

    loop = rospy.Rate(30)


    while not rospy.is_shutdown():
        loop.sleep()

    detection = String()



    #inicio YOLO



if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
