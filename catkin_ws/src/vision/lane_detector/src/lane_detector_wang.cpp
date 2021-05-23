#include"ros/ros.h"
#include"opencv/cv.h"
//#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<bits/stdc++.h>
#include "lane_detector_wang.h"
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//std::cout << "Imagen leida" << std::endl;
	namespace enc = sensor_msgs::image_encodings;
	int n_segments=5;
	int segments[5]={75/2, 140/2, 215/2, 250/2, 320/2};
	Mat img_segments[5];
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
	//cv::namedWindow(OPENCV_WINDOW);
	//cv::imshow(OPENCV_WINDOW, img->image);
	cv::Mat img = cv_ptr->image;
	cv::resize(img, img, Size(500,500));
	//imshow("lanes", img);

	int max_lowThreshold=500, max_highThreshold=500;
	Mat edges;
	//while(true)
	//{
	//edges=find_edges(img);
	cvtColor(img, img, CV_BGR2GRAY);
	medianBlur(img, img, 9);
	Canny(img, edges, 70, 150, 3 );
	//	imshow("edges", edges);

	//	char c=(char)waitKey(10);
	//	if(c=='q'||!ros::ok())
	//		break;
	//}
	extract_segments(img_segments, edges, segments, n_segments);

	int i, j, k;
	/*display edge segments
	for(i=0;i<n_segments;i++)
		cout<<img_segments[i].rows<<" "<<img_segments[i].cols<<endl;

	for(i=0;i<n_segments;i++)
  	{
  		window_name<<"edges"<<i<<"";
  		imshow(window_name.str(), img_segments[i]);
  		window_name.str("");
  	}*/
	vector<Vec4i> lines[n_segments];
	int hough_threshold[5]={30, 30, 40, 50, 50};
	int hough_minLineLength[5]={20, 25, 25, 30, 50};

	for(i=0; i<n_segments ;i++)
		{
			HoughLinesP(img_segments[i], lines[i], 1, CV_PI/180, hough_threshold[i], hough_minLineLength[i], 50 );
         for(int j=0;j<lines[i].size();j++)
          {
            //take out unwanted near-horizontal lines that spoil the votes

            double m=((double)lines[i][j][3]-lines[i][j][1])/((double)lines[i][j][2]-lines[i][j][0]);
            if((m<0.4&&m>0)||(m>-0.4&&m<0))
            {
             lines[i][j][1]=lines[i][j][3];
             lines[i][j][0]=lines[i][j][2];
             }
          }
    }
  Mat line_segments[n_segments];
  Mat empty=img;
  extract_segments(line_segments, empty, segments, n_segments);
  //Mat empty= img-img ???

  for(i=0;i<n_segments;i++)
  	for(j=0;j<lines[i].size();j++)
		{
			Vec4i l = lines[i][j];
      //cout<<lines[i][j][1]<<endl;

			line( line_segments[i], Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  	}

  Mat line=img-img;
  merge_segments(line_segments, line, segments, n_segments);
  imshow("detected lines",line);

	for(i=0;i<n_segments;i++)
  		cout<<"#lines for "<<i<<": "<<lines[i].size()<<endl;

	int vanish_row_vote[2000]={0};
	int cum_sum=500;
	for(i=4;i>=3;i--)
	{
		cum_sum-=segments[i];
		for(j=0;j<lines[i].size();j++)
			for(k=0;k<lines[i].size();k++)
			{
				if(j==k)
					continue;

				int vanish_row=find_intersection(lines[i][j], lines[i][k])+cum_sum;

				//for checking intersection function
				/*Mat ci(1000, 1000, CV_8UC3, Scalar(0));
				Mat cs[5];
				extract_segments(cs, ci, segments, n_segments);
				cv::line( cs[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
				cv::line( cs[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
				merge_segments(cs, ci, segments, n_segments);
				imshow("check", ci);
				cout<<1000-vanish_row<<endl;
				waitKey(2000);*/

				if(vanish_row>-500 && vanish_row<500)
					vanish_row_vote[500-vanish_row]++;
			}
	}
	int current_votes=0;
  int max_votes=-1, max_i=-1;
  for(i=0;i<50;i++)
  	current_votes+=vanish_row_vote[i];

	for(i=50;i<2000;i++)
  	{
  		current_votes+=vanish_row_vote[i];
  		current_votes-=vanish_row_vote[i-50];

  		if(current_votes>=max_votes)
  		{
  			max_votes=current_votes;
  			max_i=i;
  		}

  		//cout<<i<<" "<<current_votes<<endl;
  	}
	int vanish_row=max_i-25;
	cout<<"Vanishing row: "<<vanish_row<<" with votes: "<<max_votes<<endl;
	Mat output(500, 500, CV_8UC3, Scalar::all(0));
  //line.copyTo(output(cv::Rect(0, 200, 500, 500)));
	output=line(cv::Rect(0, 0, 500, 500)).clone();
  cv::line( output, Point(0, 500-vanish_row), Point(500, 500-vanish_row), Scalar(255,0,255), 10, CV_AA, 0);
  	//line( output, Point(0, 1000-vanish_row), Point(1000, 1000-vanish_row), Scalar(255,0,0), 10, CV_AA);
	imshow("output", output);
	Mat lanes(500, 500, CV_8UC3, Scalar(0));
	Mat lanes_segments[n_segments];
	extract_segments(lanes_segments, lanes, segments, n_segments);

	cum_sum=500;
	for(i=4;i>=2;i--)
  	{
  		cum_sum-=segments[i];
  		for(j=0;j<lines[i].size();j++)
  			for(k=0;k<lines[i].size();k++)
  			{
  				if(j==k)
  					continue;

  				int vanishRow=find_intersection(lines[i][j], lines[i][k])+cum_sum;

  				if(500-vanishRow>= vanish_row-20 && 500-vanishRow<= vanish_row+20)
  				{
  					if(i==4)
  						//cout<<"yay"<<j<<endl;


  					cv::line( lanes_segments[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( lanes_segments[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					/*imshow("please", lanes_segments[i]);
  					Mat temp(segments[i], 1000, CV_8UC3, Scalar(0));
  					cv::line( temp, Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( temp, Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					imshow("verify", temp);
  					waitKey(2000);*/

  				}
  			}
  	}
	merge_segments(lanes_segments, lanes, segments, n_segments);
	imshow("lanes", lanes);
	cum_sum=500;
	Mat lanes_segmentos[n_segments];
	Mat lineas(500, 500, CV_8UC3, Scalar(0));
	extract_segments(lanes_segmentos, lineas, segments, n_segments);
	for(i=4;i>=3;i--)
  	{
			double m1, c1, M1, C1, M2, C2;
			int r_dist=250;
			int l_dist=250;
			Vec4i L1;
			Vec4i L2;
  		//cum_sum-=segments[i];
			cum_sum-=segments[i];
  		for(j=0;j<lines[i].size();j++)
			{
				Vec4i l1 = lines[i][j];
				if(l1[2]-l1[0]!=0)
					m1=((double)l1[3]-l1[1])/((double)l1[2]-l1[0]);
				c1=(double)l1[3]-m1*l1[2];

				if(m1!=0)
				{
					if(m1<0)
					{
						//cout<<"xi"<<l1[0]<<endl;
						if((l_dist > (250-l1[0]))&&(250>l1[0]))
						{
							l_dist = 250-l1[0];
							L1 = l1;
							M1 = m1;
							C1 = c1;
						}
					}
					if(m1>0)
					{
						if((r_dist > (l1[0]-250))&&(l1[0]>250))
						{
							r_dist = l1[0]-250;
							L2 = l1;
							M2 = m1;
							C2 = c1;
						}
					}
				}

			}
			double xi=(C1-C2)/(M2-M1);
			double yi=M2*xi+C2;
			int Pm = (L2[0]-L1[0])/2;
			//cout<<"xi"<<xi<<endl;
			//cout<<"yi"<<yi<<endl;
			cv::line( lanes_segmentos[i], Point(xi, yi), Point(Pm+L1[0],500), Scalar(255,0,0), 3, CV_AA, 0);
			//cout<<"L1 found"<<L1<<endl;
			//cout<<"L2 found"<<L2<<endl;
			cv::line( lanes_segmentos[i], Point(L1[0], L1[1]), Point(L1[2], L1[3]), Scalar(255,0,0), 3, CV_AA, 0);
			cv::line( lanes_segmentos[i], Point(L2[0], L2[1]), Point(L2[2], L2[3]), Scalar(255,0,0), 3, CV_AA, 0);
  	}
	merge_segments(lanes_segmentos, lineas, segments, n_segments);
	//cv::line(lineas, Point(250, 500), Point(250, 0), Scalar(255,0,0), 3, CV_AA, 0);
	imshow("lanes1", lineas);
  //for(i=500-vanish_row;i>=0;i--)
  	//for(j=0;j<img.cols;j++)
  		//lanes.at<Vec3b>(i, j)={0, 0, 0};

  //for(i=0;i<img.rows;i++)
  	//for(j=0;j<img.cols;j++)
  		//if(lanes.at<Vec3b>(i, j)[0]==255)
  			//img.at<Vec3b>(i, j)={255, 0, 0};
	imshow("img", img);
	imshow("lanes", lanes);
	cv::waitKey(1);
}

int main(int argc, char** argv)
{
	std::cout << "Prueba Wang cpp" << std::endl;
	ros::init(argc, argv, "wang");
	ros::NodeHandle n("~");

	ros::Subscriber sub = n.subscribe("/app/camera/rgb/image_raw", 1000, chatterCallback);

	ros::Rate loop(30);

	while(ros::ok())
	{
		ros::spinOnce();
		loop.sleep();
	}
	return 0;

}
