#!/usr/bin/env python  
import roslib
roslib.load_manifest('marina')
 
import rospy
import tf
import turtlesim.msg
import tf.msg
import geometry_msgs.msg

pi=3.14159265359
distance=0.40


class FixedTFBroadcaster:
 
  def __init__(self):
    self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

    while not rospy.is_shutdown():
      # Run this loop at about 10Hz
      rospy.sleep(0.1) 

      self.origintomarkers()
      

  def origintomarkers(self):

    #transformation marker_0 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_0"
    t.transform.translation.x = 2*distance 
    t.transform.translation.y = distance
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi/2)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_1 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_1"
    t.transform.translation.x = distance 
    t.transform.translation.y = 2*distance
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_2 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_2"
    t.transform.translation.x = 0.0 
    t.transform.translation.y = 2*distance 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, 3*pi/2)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_3 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_3"
    t.transform.translation.x = distance 
    t.transform.translation.y = distance 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_4 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_4"
    t.transform.translation.x = 0.0 
    t.transform.translation.y = distance 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, 3*pi/2)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_5 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_5"
    t.transform.translation.x = 0.0 
    t.transform.translation.y = 0.0 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi/2)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_6 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_6"
    t.transform.translation.x = 2*distance 
    t.transform.translation.y = 2*distance
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_7 to origin
    t= geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_7"
    t.transform.translation.x = distance 
    t.transform.translation.y = 0.0 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    #transformation marker_8 to origin
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "origin"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "marker_8"
    t.transform.translation.x = 2*distance
    t.transform.translation.y = 0.0 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, pi)
    t.transform.rotation.x = angles[0]
    t.transform.rotation.y = angles[1]
    t.transform.rotation.z = angles[2]
    t.transform.rotation.w = angles[3]
    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

 
if __name__ == '__main__':
  rospy.init_node('my_tf_broadcaster')
  tfb = FixedTFBroadcaster()
  rospy.spin()