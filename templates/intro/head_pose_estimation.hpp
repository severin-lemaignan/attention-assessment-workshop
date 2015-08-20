#ifndef __HEAD_POSE_ESTIMATION
#define __HEAD_POSE_ESTIMATION

#include <opencv2/core/core.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include <vector>
#include <string>

class HeadPoseEstimation {

public:

    HeadPoseEstimation(const std::string& face_detection_model);

    void update(cv::Mat image);

    cv::Mat _debug;

private:

    dlib::cv_image<dlib::bgr_pixel> current_image;

    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;

    std::vector<dlib::rectangle> faces;

    std::vector<dlib::full_object_detection> shapes;

};

#endif // __HEAD_POSE_ESTIMATION
