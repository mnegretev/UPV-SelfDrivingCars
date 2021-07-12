#include<fstream>
#include<sstream>
#include<iostream>

#include"ros/ros.h"
#include"opencv/cv.h"
#include <opencv2/dnn/dnn.hpp>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<bits/stdc++.h>

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;
using namespace dnn;

float conf_threshold = 0.5;

// nms threshold
float nms = 0.4;
int width = 416;
int height = 416;

vector<string> classes;

// remove unnecessary bounding boxes
void remove_box(Mat&frame, const vector<Mat>&out);

// draw bounding boxes
void draw_box(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

// get output layers
vector<String> getOutputsNames(const Net& net);

//static const std::string OPENCV_WINDOW = "Image window";

String configuration = "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/yolov2-tiny.cfg";
String model = "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/yolov2-tiny.weights";
//String model = "yolov2-tiny.weights";
string classesFile = "/home/pp/UPV-SelfDrivingCars/catkin_ws/src/vision/lane_detector/scripts/coco.names";
// Load the network
Net net;

std::vector<string> layersNames;
vector<int> outLayers;
vector<String> names;

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::startWindowThread();
	//std::cout << "Imagen leida" << std::endl;
	namespace enc = sensor_msgs::image_encodings;

	std::stringstream window_name;

	cv_bridge::CvImagePtr cv_ptr;
	try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


	cv::Mat blob;

	cv::Mat frame = cv_ptr->image;

	cv::resize(frame,frame,Size(416,416));

	//cv::resize(img, frame, Size(600,500));


	// convert image to blob
	blobFromImage(frame, blob, 1/255, (cvSize(frame.size().width,frame.size().height)),Scalar(0,0,0), true, false);
	net.setInput(blob);

	cv:Mat outs;
	//names = net.getOutputsNames(net);

	std::cout << "frame size rows: " << frame.rows << '\n';
	std::cout << "frame size cols: " << frame.cols << '\n';
	try
	{
		net.forward(outs, getOutputsNames(net));
		//Mat output = net.forward();
	}
	catch(...){

	}



	static const string kWinName = "Deep learning object detection in OpenCV";

	imshow(kWinName, frame);

	cv::waitKey(1);

}

int main(int argc, char** argv)
{
	std::cout << "Prueba Yolo" << std::endl;
	std::cout << "OpenCV version : " << CV_VERSION << endl;
	ros::init(argc, argv, "yolocpp");
	ros::NodeHandle n("~");

	net = readNetFromDarknet(configuration, model);

	//Get the indices of the output layers, i.e. the layers with unconnected outputs
	outLayers = net.getUnconnectedOutLayers();
	//std::cout <<  outLayers << % <<'\n';
	//get the names of all the layers in the network
	layersNames = net.getLayerNames();

	// Get the names of the output layers in names
	//names.resize(outLayers.size());
	//for (size_t i = 0; i < outLayers.size(); ++i)
	//{
	//	names[i] = layersNames[outLayers[i] - 1];
	//}


	ros::Subscriber sub = n.subscribe("/app/camera/rgb/image_raw", 1000, chatterCallback);

	ros::Rate loop(30);

	while(ros::ok())
	{
		ros::spinOnce();
		loop.sleep();
	}
	return 0;

}


void remove_box(Mat& frame, const vector<Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > conf_threshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, conf_threshold, nms, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        draw_box(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

// Draw the predicted bounding box
void draw_box(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
}

// Get the names of the output layers
vector<Mat> outs;
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
