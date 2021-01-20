#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include "seek.h"
#include "std_msgs/UInt16.h"

int main(int argc, char** argv)
{
  // ROS_INFO("%s", msg.data.c_str());
  ROS_INFO("00");

//   // Check if video source has been passed as a parameter
//   if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "seek_ros");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("seek/image", 100);
  ros::Publisher pubDevTemper = nh.advertise<std_msgs::UInt16>("seek/device_temperature_raw",100);

//   // Convert the passed as command line parameter index for the video device to an integer
//   std::istringstream video_sourceCmd(argv[1]);
//   int video_source;
//   // Check if it is indeed a number
//   if(!(video_sourceCmd >> video_source)) return 1;

//   cv::VideoCapture cap(video_source);
//   // Check if video device can be opened with the given index
//   if(!cap.isOpened()) return 1;
    LibSeek::SeekThermalPro seek("");
    // cv::Mat frame, grey_frame;
    cv::Mat frame, frame_proc;

    if (!seek.open()) {
        std::cout << "failed to open seek cam" << std::endl;
        return -1;
    }

//   cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(30);
  ROS_INFO("01");

  std_msgs::UInt16 msgDevTemper;

  while (nh.ok()) {
    ROS_INFO("02");
    // cap >> frame;
    if (!seek.read(frame)) {
        std::cout << "no more LWIR img" << std::endl;
        return -1;
    }

    // cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);
    cv::rotate(frame, frame_proc, cv::ROTATE_90_COUNTERCLOCKWISE);

    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
    //   msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, frame_proc).toImageMsg();
    msgDevTemper.data = seek.device_temp_sensor();
    pub.publish(msg);
    pubDevTemper.publish(msgDevTemper);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// int main(int argc, char** argv)
// {
//   // ROS_INFO("%s", msg.data.c_str());
//   ROS_INFO("00");

//   // Check if video source has been passed as a parameter
//   if(argv[1] == NULL) return 1;

//   ros::init(argc, argv, "image_publisher");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);

//   // Convert the passed as command line parameter index for the video device to an integer
//   std::istringstream video_sourceCmd(argv[1]);
//   int video_source;
//   // Check if it is indeed a number
//   if(!(video_sourceCmd >> video_source)) return 1;

//   cv::VideoCapture cap(video_source);
//   // Check if video device can be opened with the given index
//   if(!cap.isOpened()) return 1;
//   cv::Mat frame;
//   sensor_msgs::ImagePtr msg;

//   ros::Rate loop_rate(5);
//   ROS_INFO("01");

//   while (nh.ok()) {
//     ROS_INFO("02");
//     cap >> frame;
//     // Check if grabbed frame is actually full with some content
//     if(!frame.empty()) {
//       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//       pub.publish(msg);
//       cv::waitKey(1);
//     }

//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }

// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_publisher");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);

//   cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

//   ros::Rate loop_rate(5);
//   while (nh.ok()) {
//     pub.publish(msg);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }

