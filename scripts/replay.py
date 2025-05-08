#!/usr/bin/env python3

import rospy
import rosbag
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rospy import Time
import sys

bag_file = ""

def republish_with_continuous_time():
    rospy.init_node("bag_timestamp_modifier", anonymous=True)
    publishers = {}
    did_one_loop = False
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

                # Publish message
                publishers[topic].publish(msg)


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