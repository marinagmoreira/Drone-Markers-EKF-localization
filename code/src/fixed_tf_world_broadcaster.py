#!/usr/bin/env python  
import roslib
roslib.load_manifest('marina')
 
import rospy
import tf
import turtlesim.msg
import tf.msg
import geometry_msgs.msg


pi=3.14159265359
distance=0.25


class FixedTFBroadcaster:
 
  def __init__(self):
    self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

    while not rospy.is_shutdown():
      # Run this loop at about 10Hz
      rospy.sleep(0.1) 

      self.origintoworld()



  def origintoworld(self):
    #transformation ar_marker_5 to world
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "origin"
    t.transform.translation.x = 0.0 
    t.transform.translation.y = 0.0 
    t.transform.translation.z = 0.0

    angles=tf.transformations.quaternion_from_euler(0, 0, -pi/2)
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