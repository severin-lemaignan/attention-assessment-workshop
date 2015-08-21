#!/usr/bin/env python  
import rospy
import std_msgs.msg

import math
import tf

listener = None

FOA = 15. / 180 * math.pi # field of attention, in radians
OBSERVER_FRAME = "/face_0"

def is_in_fov(frame):

    try:
        trans, rot = listener.lookupTransform(OBSERVER_FRAME, frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("Fail to transform: %s" %str(e))
        return False

    x, y, z = trans

    # TODO: return True when the given frame is in the field of view

    return False



if __name__ == '__main__':
    rospy.init_node('current_focus')

    listener = tf.TransformListener()

    in_focus = rospy.Publisher('current_focus', std_msgs.msg.String, queue_size=1)

    rate = rospy.Rate(10.0)

    rospy.loginfo("Waiting until a face becomes visible...")

    while not rospy.is_shutdown():
        try:
            listener.waitForTransform("/base_link", OBSERVER_FRAME,
                                      rospy.Time.now(), rospy.Duration(3.0))
            rospy.loginfo("Face detected! Starting to publish its current focus")
            break
        except tf.Exception:
            rospy.logwarn("Still no face visible...")
            rate.sleep()


    while not rospy.is_shutdown():

        # TODO: publish on topic 'current_focus' the list of frame in FoA
        for frame in listener.getFrameStrings():

            # skip ourselves
            if OBSERVER_FRAME.endswith(frame):
                continue

            rospy.loginfo("%s %s in FOA." % (frame, "is" if is_in_fov(frame) else "isn't"))

        rate.sleep()
