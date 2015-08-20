#include <cmath>
#include <ctime>

#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "head_pose_estimation.hpp"

using namespace dlib;
using namespace std;
using namespace cv;

/** Converts dlib's points to OpenCV ones
 */
inline Point2f toCv(const dlib::point& p)
{
    return Point2f(p.x(), p.y());
}


HeadPoseEstimation::HeadPoseEstimation(const string& face_detection_model)
{
        // Load face detection and pose estimation models.
        detector = get_frontal_face_detector();
        deserialize(face_detection_model) >> pose_model;
}


void HeadPoseEstimation::update(cv::Mat image)
{

    // Converts an OpenCV image (-> the webcam) to dlib's format
    current_image = cv_image<bgr_pixel>(image);

    // Detects faces
    faces = detector(current_image);

    // Find the pose of each face.
    shapes.clear();
    for (auto face : faces){
        shapes.push_back(pose_model(current_image, face));
    }

    // Draws the contours of the face and IDs of the 68 face features onto the
    // image
    _debug = image.clone();

    auto color = Scalar(0,128,128);
    auto textcolor = Scalar(255,255,255);

    for (unsigned long i = 0; i < shapes.size(); ++i)
    {
        const full_object_detection& d = shapes[i];

        for (unsigned long i = 1; i <= 16; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);

        for (unsigned long i = 28; i <= 30; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);

        for (unsigned long i = 18; i <= 21; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        for (unsigned long i = 23; i <= 26; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        for (unsigned long i = 31; i <= 35; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(30)), toCv(d.part(35)), color, 2, CV_AA);

        for (unsigned long i = 37; i <= 41; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(36)), toCv(d.part(41)), color, 2, CV_AA);

        for (unsigned long i = 43; i <= 47; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(42)), toCv(d.part(47)), color, 2, CV_AA);

        for (unsigned long i = 49; i <= 59; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(48)), toCv(d.part(59)), color, 2, CV_AA);

        for (unsigned long i = 61; i <= 67; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(60)), toCv(d.part(67)), color, 2, CV_AA);

        for (auto i = 0; i < 68 ; i++) {
            putText(_debug, to_string(i), toCv(d.part(i)), FONT_HERSHEY_DUPLEX, 0.5, textcolor);
        }
    }
}

