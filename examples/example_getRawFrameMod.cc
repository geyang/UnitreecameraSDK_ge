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
#include <stdlib.h>
#include <stdio.h>
#include <netinet/in.h>
#define PORT 8080


int main(int argc, char *argv[]){
    
    int deviceNode = 0; // default 0 -> /dev/video0
    cv::Size frameSize(1856, 800); // defalut image size: 1856 X 800
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
   

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = { 0 };
    //char* hello = "Hello from server";

    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
	    perror("socket failed");
	    exit(1);
    }


    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  (socklen_t*)&addrlen))
        < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    //int clientSock = socket(AF_UNSPEC, SOCK_DGRAM, 0);
    //int binding = bind(clientSock, INADDR_ANY, \

    std::cout << "startcap\n";

    cam.startCapture();            ///< start camera capturing

    while(cam.isOpened())
    {
        
        cv::Mat frame;
        std::chrono::microseconds t;
        if(!cam.getRawFrame(frame, t)){ ///< get camera raw image
            usleep(1000);
            continue;
        }

	std::cout << "Got camera image!" << std::endl;

        cv::Mat left,right;
        frame(cv::Rect(0, 0, frame.size().width/2, frame.size().height)).copyTo(right);
        frame(cv::Rect(frame.size().width/2,0, frame.size().width/2, frame.size().height)).copyTo(left);
        cv::hconcat(left, right, frame); 
	//cv::imshow("UnitreeCamera_Left-Right", frame);
        char key = cv::waitKey(10);
        //if(key == 27) // press ESC key
        //   break;

        int imgSize = frame.total() * frame.elemSize();

	//send data here
	send(new_socket, frame.data, imgSize, 0);
    }
    std::cout << "camera closed" << std::endl;

    cam.stopCapture();  ///< stop camera capturing
    
    return 0;
}
