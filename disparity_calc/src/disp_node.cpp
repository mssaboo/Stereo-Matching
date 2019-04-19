#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sstream>

using namespace sensor_msgs;
using namespace cv;
using namespace std;


int square(int x){
	return x*x;
}

int main(int argc, char **argv)
{
	// Initialization
	ros::init(argc, argv, "disp_calc");
	ros::NodeHandle nh;
	ros::Time t;

	// Getting parameters
	std::string package_path = ros::package::getPath("disparity_calc");
	std::string file_path_l;
	std::string file_path_r;
	float frequency = 30;
	nh.param<std::string>("file_l", file_path_l, package_path+"/src/left.png");
	nh.param("frequency", frequency, (float)30);	// in hz
	nh.param<std::string>("file_r", file_path_r, package_path+"/src/right.png");
	nh.param("frequency", frequency, (float)30);	// in hz

	// Creating image publisher object using image_transport
	image_transport::ImageTransport it_file_l(nh);
	image_transport::Publisher pub_file_l = it_file_l.advertise("/image_l", 1);

	image_transport::ImageTransport it_file_r(nh);
	image_transport::Publisher pub_file_r = it_file_r.advertise("/image_r", 1);


	// Reading the image and converting it to a ROS message
	cv_bridge::CvImage cvb_image_l;
	cvb_image_l.image = cv::imread(file_path_l);	// note: this defaults to BGR8
	sensor_msgs::Image ros_image_l = *(cvb_image_l.toImageMsg());
	ros_image_l.header.frame_id = "image_from_file_l";
	ros_image_l.encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImage cvb_image_l_gray;
	cv::cvtColor(cvb_image_l.image,cvb_image_l_gray.image,CV_BGR2GRAY);


	// Reading the image and converting it to a ROS message
	cv_bridge::CvImage cvb_image_r;
	cvb_image_r.image = cv::imread(file_path_r);	// note: this defaults to BGR8
	sensor_msgs::Image ros_image_r = *(cvb_image_r.toImageMsg());
	ros_image_r.header.frame_id = "image_from_file_r";
	ros_image_r.encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImage cvb_image_r_gray;
	cv::cvtColor(cvb_image_r.image,cvb_image_r_gray.image,CV_BGR2GRAY);




		Mat left,right,disp_img;

		left = cvb_image_l.image;
		right = cvb_image_r.image;

		disp_img = Mat::zeros(left.rows, left.cols, CV_8UC1);

		int ROW=left.rows;
		int COL=left.cols;

		// stores the disparity and sum of squared distance (SSD)
		vector <vector <int> > disp;
	        disp.resize(left.rows, vector <int> (left.cols, 0));

		vector <vector <int> > SSD_value;
	        SSD_value.resize(left.rows, vector <int> (left.cols, 0));

		int disparity_min = -100, disparity_max = 100;
		int disparity_range = disparity_max - disparity_min;
		int half_block_size = 1;
		int MAX_DISP=-1, MIN_DISP=1000000;


		int SSD=0, l_r, l_c, r_r, r_c;

		for(int i=0; i<ROW; i++)
		{
			for(int j=0; j<COL; j++)
			{
				disp[i][j]=0;
				SSD_value[i][j]=100000000;
			}
		}

		cout<<"Calculating Disparity"<<endl;

		for(int i=0+half_block_size; i<left.rows-half_block_size; i++)
		{
			for(int j=0+half_block_size; j<left.cols-half_block_size; j++)
			{

				for(int range=disparity_min; range<=disparity_max; range++)
				{
					SSD = 0;

					for(l_r = -half_block_size + i; l_r <= half_block_size + i; l_r++)
					{
						for(l_c = -half_block_size + j; l_c <= half_block_size + j; l_c++)
						{
						r_r = l_r;
						r_c = l_c + range;

						SSD += square(left.at<cv::Vec3b>(l_r,l_c)[0] - right.at<cv::Vec3b>(r_r, min(max(0, r_c), COL-1))[0])
							+ square(left.at<cv::Vec3b>(l_r,l_c)[1] - right.at<cv::Vec3b>(r_r, min(max(0, r_c), COL-1))[1])
							+ square(left.at<cv::Vec3b>(l_r,l_c)[2] - right.at<cv::Vec3b>(r_r, min(max(0, r_c), COL-1))[2]);
						}
					}

					if(SSD < SSD_value[i][j])
					{
						disp[i][j] = range;
						SSD_value[i][j] = SSD;
					}
				}


			}
			cout<<left.rows-half_block_size-i<<" iterations left"<<endl;


			for(int p=0; p<ROW; p++)
			{
				for(int j=0; j<COL; j++)
				{
					disp_img.at<uchar>(p,j) = 63+(int)192.0*(disp[p][j]-disparity_min)/disparity_range;
					if(MAX_DISP < disp[p][j])
						MAX_DISP = disp[p][j];
					if(disp[p][j] < MIN_DISP)
						MIN_DISP = disp[p][j];
				}
			}
			//imshow("disparity", disp_img);

		}

		cout << "MAX_DISP:" << MAX_DISP << endl;
		cout << "MIN_DISP:" << MIN_DISP << endl;

//		for(int i=0; i<ROW; i++)
//		{
//			for(int j=0; j<COL; j++)
//			{
//				disp_img.at<uchar>(i,j) = 63+(int)192.0*(disp[i][j]-disparity_min)/disparity_range;
//			}
//		}
		cout<<"SSD Image saved"<<endl;
		imwrite("disparity_ssd.png", disp_img);



	cv::Ptr<StereoBM> leftSBM = StereoBM::create(64,7);
	Mat disp1, disp8;
	leftSBM->compute( cvb_image_l_gray.image, cvb_image_r_gray.image, disp1 );
	normalize(disp1, disp8, 0, 255, CV_MINMAX, CV_8U);
  cv::imwrite("disparity_inbuilt.png",disp8);
	cout<<"Inbuilt code image saved"<<endl;




	// Creating image publisher object using image_transport
	image_transport::ImageTransport it_file_disp_ssd(nh);
	image_transport::Publisher pub_file_disp_ssd = it_file_disp_ssd.advertise("/image_disp_ssd", 1);

	image_transport::ImageTransport it_file_disp_ib(nh);
	image_transport::Publisher pub_file_disp_ib = it_file_disp_ib.advertise("/image_disp_ib", 1);


	// Reading the image and converting it to a ROS message
	cv_bridge::CvImage cvb_image_disp_ssd;
	cvb_image_disp_ssd.image = cv::imread("disparity_ssd.png");	// note: this defaults to BGR8
	sensor_msgs::Image ros_image_disp_ssd = *(cvb_image_disp_ssd.toImageMsg());
	ros_image_disp_ssd.header.frame_id = "image_from_file_disp_ssd";
	ros_image_disp_ssd.encoding = sensor_msgs::image_encodings::BGR8;


	// Reading the image and converting it to a ROS message
	cv_bridge::CvImage cvb_image_disp_ib;
	cvb_image_disp_ib.image = cv::imread("disparity_inbuit.png");	// note: this defaults to BGR8
	sensor_msgs::Image ros_image_disp_ib = *(cvb_image_disp_ib.toImageMsg());
	ros_image_disp_ib.header.frame_id = "image_from_file_disp_ib";
	ros_image_disp_ib.encoding = sensor_msgs::image_encodings::BGR8;



	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		t = ros::Time::now();
		ros_image_l.header.stamp = t;	// all other values remain constant
		ros_image_r.header.stamp = t;
		pub_file_l.publish(ros_image_l);
		pub_file_r.publish(ros_image_r);
		ros_image_disp_ssd.header.stamp = t;	// all other values remain constant
		ros_image_disp_ib.header.stamp = t;
		pub_file_disp_ssd.publish(ros_image_disp_ssd);
		pub_file_disp_ib.publish(ros_image_disp_ib);

		ros::spinOnce();
		loop_rate.sleep();
	}




	return 0;
}
