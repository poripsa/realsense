#pragma once

#include "../include/base_realsense_node.h"
#include "../include/librealsense2/h/rs_option.h"

using namespace realsense_ros_camera;

class Temporal : public BaseRealSenseNode
{
public:
    Temporal(ros::NodeHandle& nodeHandle,
              ros::NodeHandle& privateNodeHandle,
              rs2::device dev, const std::string& serial_no);
private:
    void getImage(uchar *image);
    void returnAverage();
};
