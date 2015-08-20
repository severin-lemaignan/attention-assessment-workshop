#include <string>
#include <set>

// our head pose library
#include "head_pose_estimation.hpp"

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>


class ROSHeadPoseEstimator
{
public:

    ROSHeadPoseEstimator(ros::NodeHandle& rosNode,
                         const std::string& modelFilename,
                         const std::string& cameraFrame);

private:

    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;

    ros::Publisher nb_detected_faces_pub;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string cameraFrame;

    cv::Mat inputImage;
    HeadPoseEstimation estimator;

    void detectFaces(const sensor_msgs::ImageConstPtr& msg,
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
};

