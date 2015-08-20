
Workshop Material: From a 2D face to attention assessment
=========================================================

Abstract
--------

Assessing in real-time the focus of attention of a human interacting with a
robot is essential to understand implicit references in a dialogue ("robot, take
that!"), to measure engagement (is the human "with me"?) or to detect outright
problems (why is this human staring at me since 20 minutes?)

With a regular camera, a bit of math and a good face detector, we can actually
estimate pretty accurately and in real-time the 3D head pose of surrounding
humans. Combined with some frames' magic, this lets us assess what the human is
looking at.

During the workshop, we will:

- use a state-of-art open-source face detector (coming for the dlib library) and
  a PnP algorithm (from OpenCV) to match a 3D template of a head onto a 2D
  camera stream,
- export the head pose as a ROS tf frame and see you face in RViz,
- write a dedicated ROS node that computes what is seen by the human at a given
  time,
- test the system in a pre-recorded scenario,
- (and if you are fast enough,) validate the approach by manually annotating the
  focus of attention and computing an inter-judge agreement between the robot
  and you.

Prerequisites
-------------

Since the workshop is packed, please make sure that the following prerequisites
are met *beforehand*.

- The workshop uses C++ and Python. You need to have a working knowledge of
  both,
- I assume a working installation of ROS (Indigo or Jade) and some basic
  knowledge of it (including topics, RViz and tf). Typically, you should be
  comfortable with the material covered in the "Introduction to tf" tutorial
  (http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf),
- please download dlib (http://dlib.net/) and make sure that the face detection
  example compiles and works (-> dlib/examples/face_landmark_detection_ex.cpp),
- make sure OpenCV 2.4 is installed and working (this should have come with ROS
  anyway). If you have never used OpenCV before, I recommend you to have a look
  to some of its tutorials: http://docs.opencv.org/doc/tutorials/tutorials.html,
- optionally, calibrate your webcam beforehand with the OpenCV calibration tool.
  This is not mandatory, but will allow a better accuracy.





