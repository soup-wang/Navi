#!/usr/bin/env python
# encoding=utf-8
import turtlesim.msg
import tf
import rospy
import roslib

from sensor_msgs.msg import Imu
import math

# roslib.load_manifest('learning_tf')

# 使用 tf 完成坐标变换，订阅这个topic "turtleX/pose" 并且对每一条msg进行以下计算


def imu_callback(msg):

    # pitch=180∗atan2(accelX,sqrt(accelY∗accelY+accelZ∗accelZ))/PI;
    # roll=180∗atan2(accelY,sqrt(accelX∗accelX+accelZ∗accelZ))/PI;

    rollxyz = msg.linear_acceleration
    roll = math.atan2(rollxyz.y, math.sqrt(
        rollxyz.x ** 2+rollxyz.y ** 2+rollxyz.z ** 2))
    pitch = math.atan2(rollxyz.x, math.sqrt(
        rollxyz.x ** 2+rollxyz.y ** 2+rollxyz.z ** 2))

    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(roll, -pitch, 0),
                     rospy.Time.now(),
                     "world",
                     "base_link")  #发布乌龟的平移和翻转


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')  # 添加新的节点

    rospy.Subscriber(
        "/mavros/imu/data", Imu, imu_callback)
    rospy.spin()
