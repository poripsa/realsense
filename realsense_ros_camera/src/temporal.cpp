#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

#include "pluginlib/class_list_macros.h"
#include <nodelet/nodelet.h>
#include "../include/temporal.h"
#include "../include/librealsense2/h/rs_option.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <queue>

using namespace realsense_ros_camera;

Temporal::Temporal(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, rs2::device dev, const std::string& serial_no)
    : BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no)
{}

sensor_msgs::ImagePtr Temporal::getImage(uchar *image_data)
{
    //BaseRealSenseNode *image;
    //const ros::Time& t;
    //image->publishPCTopic(t,  true);
    //sensor_msgs::PointCloud2 msg_pointcloud;
    //_sensors[DEPTH];
    auto depth_intrinsics = _stream_intrinsics[DEPTH];
    //auto image_depth = _image[DEPTH].data;
    //IplImage * result;
    auto result = cvCreateImage(cvSize(depth_intrinsics.width, depth_intrinsics.height), IPL_DEPTH_8U, 3);

    //std::queue<uchar> depth_image_queue;
    uchar depth_arr[6];

    for (int i = 0; i<=6; i++)
    {
        //depth_image_queue.emplace(image_depth);
        depth_arr[i] = image_data;
    }

    //cv2.add(), cv2.addWeighted()
    //dst = cv2.addWeighted(img1,0.7,img2,0.3,0);

    cv::addWeighted(image_data[0], 0.3, image_data[1], 0.3, result, -1);
    cv::addWeighted(result, 0.6, image_data[2],0.1, result, -1);
    cv::addWeighted(result, 0.7, image_data[3],0.1, result, -1);
    cv::addWeighted(result, 0.8, image_data[4],0.1, result, -1);
    cv::addWeighted(result, 0.9, image_data[5],0.1, result, -1);
    //return result;

    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(std_msgs::Header(), _encoding[stream], image).toImageMsg();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = _optical_frame_id[stream];
    img->header.stamp = t;
    img->header.seq = _seq[stream];

    return img;


}

void Temporal::returnAverage()
{
    //publishFrame


}







