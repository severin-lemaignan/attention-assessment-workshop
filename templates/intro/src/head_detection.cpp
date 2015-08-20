#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "head_pose_estimation.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if(argc < 2) {
        cerr << "Usage: " << endl <<
                "head_detection model.dat (eg: head_detection ../share/shape_predictor_68_face_landmarks.dat)" << endl;
        return 1;
    }

    namedWindow("headdetection");

    // The class is called 'HeadPoseEstimation', but for know
    // it just detects the head and extracts its features.
    auto detector = HeadPoseEstimation(argv[1]);

    Mat frame;

    // Configure the video capture
    VideoCapture video_in(0);
    video_in.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    video_in.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

    if(!video_in.isOpened()) {
        cerr << "Couldn't open camera" << endl;
        return 1;
    }

    // Loop until ESC is pressed
    while(true) {
        video_in >> frame;

        detector.update(frame);

        imshow("headdetection", detector._debug);
        if (waitKey(10) >= 0) break;

    }
}



