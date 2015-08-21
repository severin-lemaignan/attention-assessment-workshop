#!/usr/bin/env python  
import matplotlib.pyplot as plt
import numpy as np

import rospy
import std_msgs.msg

import math
import tf

from collections import OrderedDict

OBSERVER_FRAME = "face_0"

WINDOW_WIDTH = 50
LINE_WIDTH = 0.01

frames_in_fov = []

if __name__ == '__main__':
    rospy.init_node('focus_analysis')

    rate = rospy.Rate(10.0)

    listener = tf.TransformListener()

    # Make sure the tf tree is fully published...
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform("base_link", OBSERVER_FRAME, rospy.Time.now(), rospy.Duration(3.0))
            rospy.loginfo("Face detected! Starting to plot its current focuses")
            break
        except tf.Exception:
            rospy.logwarn("Still no face visible...")
            rate.sleep()

    # Retrieve all existing tf frames
    frames = OrderedDict([(frame, []) for frame in listener.getFrameStrings() if frame != OBSERVER_FRAME])

    rospy.loginfo("Focus Analyse running. Monitored frames: %s" % str(frames))

    ### Preparing the plots
    y_pos = range(len(frames) + 1)

    fig = plt.figure()
    ax = fig.add_axes((0.1, 0.2, 0.8, 0.7))
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    plt.xticks([])
    plt.yticks(y_pos, ["no focus"] + frames.keys())
    ax.set_ylim([0, len(frames) + 0.1])
    ax.set_xlim(0, WINDOW_WIDTH)

    plt.xlabel('time')
    fig.text(0.5, 0.05, 'Focuses of attention over time', ha='center')

    plt.ion() # allows for real-time plot updates
    plt.show()

    # one plot per frame
    plots = {(name, ax.bar(range(WINDOW_WIDTH), [LINE_WIDTH] * WINDOW_WIDTH, 1)) for name, data in frames.items()}


    ###################################################
    ## Main loop
    ###################################################

    t = 0
    while not rospy.is_shutdown():

        # TODO: replace these random values by actual detections!
        for frame in frames.keys():
            frames[frame].append(1 if np.random.random() > 0.5 else 0)

        # use a sliding window of width WINDOW_WIDTH to update the plots
        for i in range(min(t, WINDOW_WIDTH)):
            offset = 0 # y pos of each frame
            for frame, plot in plots:
                offset += 1
                plot[i].set_y(frames[frame][-min(t, WINDOW_WIDTH) + i - 1] * offset)

        t += 1
        plt.draw()

        rate.sleep()
