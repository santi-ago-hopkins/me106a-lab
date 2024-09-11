import sys
import rospy
from geometry_msgs.msg import Twist

def talker():

    #take in system arg
    turtle = sys.argv[1]

    #make message object
    pub_message = rospy.Publisher(f'/{turtle}/cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(10) # 10hz

    vel_cmd = Twist()

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():

        #take user velocity input
        vel_cmd.linear.x = float(input("specify a velocity: "))
        vel_cmd.angular.z = float(input("specify an angle: "))
        
        # Publish our string to the 'chatter_talk' topic
        pub_message.publish(vel_cmd)

        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass
