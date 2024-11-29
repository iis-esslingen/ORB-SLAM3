#!/usr/bin/env python3

import os

import rospy
from geometry_msgs.msg import PoseStamped


class TrajectorySaverNode:
    def __init__(self) -> None:
        rospy.init_node("trajectory_saver_node", anonymous=True)
        rospy.on_shutdown(self.shutdown)
        topic_name = rospy.get_param("~topic_name")
        rospy.Subscriber(topic_name, PoseStamped, self._callback)

        # Create a file to write the odometry data
        filename = rospy.get_param("~file_name")
        self._file = open(filename, "w")
        self._file.write("# timestamp x y z qx qy qz qw\n")
        print("Odometry saved in: " + os.path.realpath(self._file.name))

    def _callback(self, msg: PoseStamped) -> None:
        # Process the received odometry message
        # For this example, we'll simply write the message to the file
        self._file.write(
            str(msg.header.stamp.secs)
            + "."
            + str(msg.header.stamp.nsecs)
            + " "
            + str(msg.pose.position.x)
            + " "
            + str(msg.pose.position.y)
            + " "
            + str(msg.pose.position.z)
            + " "
            + str(msg.pose.orientation.x)
            + " "
            + str(msg.pose.orientation.y)
            + " "
            + str(msg.pose.orientation.z)
            + " "
            + str(msg.pose.orientation.w)
            + "\n"
        )

    def shutdown(self):
        # Close the file before shutting down the node
        self._file.close()


if __name__ == "__main__":
    try:
        node = TrajectorySaverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()
