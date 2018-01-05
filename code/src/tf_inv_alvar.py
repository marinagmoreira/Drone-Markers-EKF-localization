#!/usr/bin/env python  
import roslib
roslib.load_manifest('marina')
import rospy
import math
import tf
from tf import transformations as t

import tf2_msgs.msg


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                 x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                 -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                 x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
    

def tf_handle(msg):

    for i in range(0,len(msg.transforms)):

        if msg.transforms[i].header.frame_id=="usb_cam":
            print("hey")


            #check it it saw marker 0----------------------------------------------------
            if (msg.transforms[i].child_frame_id=="ar_marker_0"):
                
                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_0', rospy.Time(0))
                    print("0")
                    
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_0")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 1----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_1"):

                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_1', rospy.Time(0))
                    print("1")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_1")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 2----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_2"):

                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_2', rospy.Time(0))
                    print("2")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_2")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 3----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_3"):

                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_3', rospy.Time(0))
                    print("3")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_3")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 4----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_4"):

                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_4', rospy.Time(0))
                    print("4")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_3")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 5----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_5"):
                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_5', rospy.Time(0))
                    print("5")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_5")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 6----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_6"):
                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_6', rospy.Time(0))
                    print("6")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_6")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 7----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_7"):
                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_7', rospy.Time(0))
                    print("7")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_7")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            #check it it saw marker 8----------------------------------------------------
            elif (msg.transforms[i].child_frame_id=="ar_marker_8"):
                try:
                    (trans,rot) = listener.lookupTransform('/usb_cam', '/ar_marker_8', rospy.Time(0))
                    print("8")
                    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inversed_transform = t.inverse_matrix(transform)
                    translation = t.translation_from_matrix(inversed_transform)
                    quaternion = t.quaternion_from_matrix(inversed_transform)

                    br = tf.TransformBroadcaster()
                    br.sendTransform(translation, quaternion, rospy.Time.now(),
                        "drone",
                        "marker_8")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue


if __name__ == '__main__':
    rospy.init_node('tf_invert')
    listener = tf.TransformListener()
    rate = rospy.Rate(0.5)
    rospy.Subscriber('/tf',
                      tf2_msgs.msg.TFMessage,
                      tf_handle)
    rospy.spin()