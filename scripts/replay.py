#!/usr/bin/env python3

import rospy
import rosbag
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rospy import Time
import sys
from geometry_msgs.msg import TransformStamped
import tf2_ros
import re

bag_file = ""

def publish_static_transforms():
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transforms = []

    # First static transform
    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_footprint"
    t1.child_frame_id = "base_link"
    t1.transform.translation.x = 0.0
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.0
    t1.transform.rotation.x = 0.0
    t1.transform.rotation.y = 0.0
    t1.transform.rotation.z = 0.0
    t1.transform.rotation.w = 1.0
    static_transforms.append(t1)

    # Second static transform
    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_link"
    t2.child_frame_id = "camera_link"
    t2.transform.translation.x = 0.1
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.7
    t2.transform.rotation.x = 0.0
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0
    static_transforms.append(t2)

    static_broadcaster.sendTransform(static_transforms)

def republish_with_continuous_time():
    rospy.init_node("bag_timestamp_modifier", anonymous=True)
    publishers = {}
    while not rospy.is_shutdown():
        with rosbag.Bag(bag_file, "r") as bag:
            last_t = None

            for topic, msg, t in bag.read_messages():
                if rospy.is_shutdown():
                    break

                # Sleep based on original message spacing
                if last_t is not None:
                    dt = (t - last_t).to_sec()
                    if dt > 0:
                        rospy.sleep(dt)
                last_t = t
                
                topic = re.sub(r"/camera/color", "/camera/rgb", topic)
                topic = re.sub(r"/camera/aligned_depth_to_color/image_raw", "/camera/depth/image", topic)
                topic = re.sub(r"/camera/aligned_depth_to_color/camera_info", "/camera/depth/camera_info", topic)

                if topic not in publishers:
                    latch = True if topic in ["/tf_static", "tf_static"] else False
                    publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10, latch=latch)

                # Assign fresh timestamps
                now = rospy.Time.now()
                if hasattr(msg, "header"):
                    msg.header.stamp = now
                if topic in ["/tf", "/tf_static", "tf", "tf_static"]:
                    for transform in msg.transforms:
                        transform.header.stamp = now

                publishers[topic].publish(msg)
                publish_static_transforms()


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            bag_file = sys.argv[1]
        else:
            rospy.logerr("Please provide the bag file path as a command line argument.")
            sys.exit(1)
        republish_with_continuous_time()
    except rospy.ROSInterruptException:
        pass
