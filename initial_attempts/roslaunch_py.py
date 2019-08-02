#!/usr/bin/env python
import roslaunch
import rospy
from std_msgs.msg import Int32
import sys
class ros_wheels:
    def __init__(self):
    self.subscriber = rospy.Subscriber('num_objects_detected',Int32,self.callback)

    def callback(self, data):
    rospy.loginfo("I receive %s", data.data)
        self.callback_data = data.data

def main(args):
    rw = ros_wheels()
    rospy.init_node('en_Mapping', anonymous=True)
    try:
        rospy.spin()
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/software/catkin_ws/src/mpyackage/lane_following.launch"])
            if self.callback_data == 0:
                    launch.start()
                    rospy.loginfo("started")
            #launch.shutdown()
            if self.callback_data >= 1:
                    rospy.loginfo("Can't start. Objects detected")
                    try:
                            launch.shutdown()
                    except:
                            pass

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
