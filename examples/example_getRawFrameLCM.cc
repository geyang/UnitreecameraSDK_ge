/**
  * @file example_getRawFrame.cc
  * @brief This file is part of UnitreeCameraSDK.
  * @details This example that how to get camera raw frame.
  * @author ZhangChunyang
  * @date  2021.07.31
  * @version 1.0.1
  * @copyright Copyright (c) 2020-2021, Hangzhou Yushu Technology Stock CO.LTD. All Rights Reserved.
  */

#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <lcm/lcm-cpp.hpp>
#include "camera_message_lcmt.hpp"
#include <ctime>

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int main(int argc, char *argv[]){
    
    int deviceNode = 0; // default 0 -> /dev/video0
    cv::Size frameSize(928, 400); // defalut image size: 1856 X 800
    int fps = 30;
    
    if(argc >= 2){
        deviceNode = std::atoi(argv[1]);
        if(argc >= 4){
            frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
        }
        if(argc >=5)
            fps = std::atoi(argv[4]);
    }

    std::cout << "node " << deviceNode << std::endl;

    UnitreeCamera cam(deviceNode);  ///< init camera by device node number
    if(!cam.isOpened())
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera frame rate

    std::cout << "SANITY" << std::endl;
    std::cout << "Device Position Number:" << cam.getPosNumber() << std::endl;
   

    lcm::LCM _simpleLCM("udpm://239.255.76.67:7667?ttl=255");
    
    camera_message_lcmt cam_msg_simple = {0};


    cam.startCapture();            ///< start camera capturing
    std::clock_t start;
    start = std::clock();

    while(cam.isOpened())
    {
        
        cv::Mat frame;
        std::chrono::microseconds t;
        if(!cam.getRawFrame(frame, t)){ ///< get camera raw image
            usleep(1000);
            continue;
        }

	cv::pyrDown(frame, frame, cv::Size(frame.cols/2, frame.rows/2));

	//std::cout << "Got camera image!" << std::endl;

        cv::Mat left,right;
        frame(cv::Rect(0, 0, frame.size().width/2, frame.size().height)).copyTo(right);
        frame(cv::Rect(frame.size().width/2,0, frame.size().width/2, frame.size().height)).copyTo(left);
        cv::hconcat(left, right, frame); 
	//cv::imshow("UnitreeCamera_Left-Right", frame);
        char key = cv::waitKey(10);
        //if(key == 27) // press ESC key
        //   break;

	//std::string ty =  type2str( frame.type() );
        //printf("Matrix: %s %dx%d \n", ty.c_str(), frame.cols, frame.rows );

        int imgSize = frame.total() * frame.elemSize();

	//send data here
	for(auto j = 0, k = 0; j < frame.rows; j++){
		for(auto i = 0; i < frame.cols; i++){
        		cam_msg_simple.data[k] = (uint8_t)frame.at<cv::Vec3b>(j, i)[0];
        		cam_msg_simple.data[k+frame.rows*frame.cols] = (uint8_t)frame.at<cv::Vec3b>(j, i)[1];
        		cam_msg_simple.data[k+2*frame.rows*frame.cols] = (uint8_t)frame.at<cv::Vec3b>(j, i)[2];
			k++;
		}
	}
        //std::cout << "element 0: " << unsigned((uint8_t)frame.at<cv::Vec3b>(300, 200)[0]) << std::endl;
	_simpleLCM.publish("camera" + std::to_string(cam.getPosNumber()), &cam_msg_simple);

	std::cout << "Frequency: " << (double) CLOCKS_PER_SEC / (std::clock() - start) << " Hz" << std::endl;
        start = std::clock();
        //return 0;
    }
    std::cout << "camera closed" << std::endl;

    cam.stopCapture();  ///< stop camera capturing
    
    return 0;
}

