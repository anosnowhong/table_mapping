import rospy
from geometry_msgs.msg import Vector3


# Twist the view angle
def camera_down():
    # from this angle can see more thing on a table when closer to table
    ca_pub = rospy.Publisher('/ptu', Vector3, queue_size=1)
    rospy.init_node('camera_down', anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        control = Vector3()
        control.x = 1.0
        control.y = 1.0
        control.z = 1.0
        ca_pub.publish(control)
        rate.sleep()
        rospy.signal_shutdown("This only run ONCE.")

    #rospy.sleep(1)

if __name__ == '__main__':

    camera_down()

