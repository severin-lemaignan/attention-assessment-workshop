#include <string>
#include <ros/ros.h>

#include "ros_node.hpp"

using namespace std;
using namespace cv;

ROSHeadPoseEstimator::ROSHeadPoseEstimator(ros::NodeHandle& rosNode,
                                           const string& modelFilename,
                                           const string& cameraFrame) :
            rosNode(rosNode),
            it(rosNode),
            estimator(modelFilename),
            cameraFrame(cameraFrame)
{
    sub = it.subscribeCamera("image", 1, &ROSHeadPoseEstimator::detectFaces, this);
    nb_detected_faces_pub = rosNode.advertise<std_msgs::Char>("nb_detected_faces", 1);
}

void ROSHeadPoseEstimator::detectFaces(const sensor_msgs::ImageConstPtr& msg, 
                                       const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 


    // TODO: process the image and publish one TF frame per detected face
}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "attention_assessment");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    // load parameters
    string cameraFrame;
    _private_node.param<string>("camera_frame", cameraFrame, "");

    string modelFilename;
    _private_node.param<string>("face_model", modelFilename, "");

    if (modelFilename.empty()) {
        ROS_ERROR_STREAM("You must provide the face model with the parameter face_model.\n" <<
                         "For instance, _face_model:=shape_predictor_68_face_landmarks.dat");
        return(1);
    }

    // initialize the detector by subscribing to the camera video stream
    ROS_INFO_STREAM("Initializing the face detector with the model " << modelFilename <<"...");
    ROSHeadPoseEstimator estimator(rosNode, modelFilename, cameraFrame);
    ROS_INFO("head_pose_estimator is ready. ...but it does nothing for now! You need to complete the code");
    ros::spin();

    return 0;
}

