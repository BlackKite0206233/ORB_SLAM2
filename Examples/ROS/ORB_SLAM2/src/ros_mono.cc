/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<vector>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ctime>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <signal.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"../../../include/System.h"

#include<ros/ros.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <DenseInput.h>
using namespace std;
using namespace cv;

void mySigintHandler(int s) {
	ros::shutdown();
}

ros::Publisher pose_pub; 
ros::Publisher pub_dense;
double old_max;
double old_min;
bool init = false;

string outputPath;

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System* mpSLAM;
};


vector<cv::Mat> gyroMatrices;
int gyroFrameCounter;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono");
	ros::start();

	if(argc != 5)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings path_to_gyroscope output_path" << endl;        
		ros::shutdown();
		return 1;
	}    
	outputPath = argv[4];
	mkdir(argv[4], S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	cerr << endl << "Initialization" << endl;
	// Load gyroscope matrix data here.
	gyroFrameCounter = 0;
	gyroMatrices.clear();

	ifstream gyroFile(argv[3]);	
	
	string line;
	while(getline(gyroFile, line)){
		cv::Mat gyroMat(3, 3, CV_32F);
		
		istringstream line_ss(line);
		
		for(int row = 0;row < 3;row++){
			for(int col = 0;col < 3;col++){
				float val = 0;
				line_ss >> val;
				gyroMat.at<float>(row, col) = val;
			}		
		}
		
		gyroMatrices.push_back(gyroMat);
	}

	gyroFile.close();

	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
        cerr << endl << "Done!" << endl;

	ImageGrabber igb(&SLAM);

	ros::NodeHandle nodeHandler;
	ros::Subscriber sub = nodeHandler.subscribe("/camera/rgb/image_color", 1000, &ImageGrabber::GrabImage, &igb);
	//ros::Subscriber sub = nodeHandler.subscribe("/status", 1000, mySigintHandler);
	// pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/camera_pose",1);
	// pub_dense = nodeHandler.advertise<svo_msgs::DenseInput>("/ORB/DenseInput",1);

	signal(SIGINT, mySigintHandler);
	
	ros::spin();

    // Stop all threads
	SLAM.Shutdown();

    // Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM(outputPath + "/KeyFrameTrajectory.txt");

	ros::shutdown();

	return 0;
}



void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // clock_t start, end;
    // double cpu_time_used;
    // start = clock();
    // Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_bridge::CvImageConstPtr cv_ptr_rgb;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO8);
		cv_ptr_rgb = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	//if(gyroFrameCounter >= gyroMatrices.size()){
	//	return;	
	//}

	// load gyrosopeMatrix w/ timestamp here, then pass it to TrackMonocular
	cv::Mat gyroscopeMatrix = gyroMatrices[0];//gyroFrameCounter];
	//gyroFrameCounter += 1;

	cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image, gyroscopeMatrix, cv_ptr->header.stamp.toSec());
	// if (pose.empty())
	// 	return;
	
	// cv::Mat  TWC=mpSLAM->mpTracker->mCurrentFrame.mTcw.inv();  
	// cv::Mat RWC= TWC.rowRange(0,3).colRange(0,3);  
	// cv::Mat tWC= TWC.rowRange(0,3).col(3);

	// tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
	// 	RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
	// 	RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));
	// tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));
	// tf::Quaternion q;
	// M.getRotation(q);

	// static tf::TransformBroadcaster br;
	// tf::Transform transform = tf::Transform(M, V);
	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "init_link", "camera_pose"));

	// geometry_msgs::PoseStamped _pose;
	// _pose.pose.position.x = transform.getOrigin().x();
	// _pose.pose.position.y = transform.getOrigin().y();
	// _pose.pose.position.z = transform.getOrigin().z();
	// _pose.pose.orientation.x = transform.getRotation().x();
	// _pose.pose.orientation.y = transform.getRotation().y();
	// _pose.pose.orientation.z = transform.getRotation().z();
	// _pose.pose.orientation.w = transform.getRotation().w();

	// _pose.header.stamp = ros::Time::now();
	// _pose.header.frame_id = "init_link";
	// pose_pub.publish(_pose);

	// double min_z = std::numeric_limits<double>::max();  
	// double max_z = std::numeric_limits<double>::min();  
	// bool flag = mpSLAM->mpTracker->mCurrentFrame.getSceneDepth(mpSLAM->mpTracker->mCurrentFrame,max_z,min_z);
    // //ROS_INFO("REACHED 2");
    // //ROS_INFO("Max: %f Min: %f",max_z,min_z);
	// if(flag)
	// {
	// 	old_min = min_z;
	// 	old_max = max_z;
	// 	init = true;

	// }
	// if(init)
	// {
	// 	svo_msgs::DenseInput msg_dense;
	// 	msg_dense.header.stamp = ros::Time::now();
	// 	msg_dense.header.frame_id = "world";

	// 	cv_bridge::CvImage img_msg;  
	// 	img_msg.header.stamp=msg_dense.header.stamp;  
	// 	img_msg.header.frame_id="camera";  
	// 	img_msg.image=cv_ptr_rgb->image;  

	// 	img_msg.encoding = sensor_msgs::image_encodings::BGR8;  
	// 	msg_dense.image = *img_msg.toImageMsg();
		
	// 	msg_dense.min_depth = (float)old_min;
	// 	msg_dense.max_depth = (float)old_max;

	// 	msg_dense.pose.position.x = tWC.at<float>(0,0);  
	// 	msg_dense.pose.position.y = tWC.at<float>(1,0);  
	// 	msg_dense.pose.position.z = tWC.at<float>(2,0);  
	// 	msg_dense.pose.orientation.x = q.x();
	// 	msg_dense.pose.orientation.y = q.y();
	// 	msg_dense.pose.orientation.z = q.z();
	// 	msg_dense.pose.orientation.w = q.w();

	// 	pub_dense.publish(msg_dense); 
	// }
    // end = clock();
    // cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    //ROS_INFO_STREAM("exe time: " << cpu_time_used << "\n");
}


