

#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char *argv[]){

    int deviceNode = 0; // default 0 -> /dev/video0
//    cv::Size frameSize(1856, 800); // defalut image size: 1856 X 800
    int fps = 30;
//
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_depth", 1);
    //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    //cv::waitKey(30);
//
//    std::cout << "Opening cam" << deviceNode << std::endl;
//
//
    if(argc >= 2){
        deviceNode = std::atoi(argv[1]);
//        if(argc >= 4){
//            frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
//        }
//        if(argc >=5)
//            fps = std::atoi(argv[4]);
    }

//    UnitreeCamera cam(deviceNode);  ///< init camera by device node number
    UnitreeCamera cam("../stereo_camera_config.yaml");
    if(!cam.isOpened())
        exit(EXIT_FAILURE);

    std::cout << "Device Position Number:" << cam.getPosNumber() << std::endl;

    cam.startCapture();            ///< start camera capturing
    cam.startStereoCompute(); ///< start disparity computing

    ros::Rate r(fps);

    while(cam.isOpened())
    {

        cv::Mat depth;
        std::chrono::microseconds t;
        if(!cam.getDepthFrame(depth, true, t)){  ///< get stereo camera depth image
            usleep(1000);
            continue;
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth).toImageMsg();
        pub.publish(msg);



        ros::spinOnce();
        //std::cout << "published\n";
        r.sleep();


//        char key = cv::waitKey(10);
//        if(key == 27) // press ESC key
//           break;
    }

    cam.stopStereoCompute();  ///< stop disparity computing
    cam.stopCapture();  ///< stop camera capturing

    return 0;
}
