#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def publish_timer():
    rospy.init_node('timer_publisher', anonymous=True)
    
    # Create a publisher on the '/timer' topic, publishing Float64 messages
    timer_pub = rospy.Publisher('/Timing', Float64, queue_size=10)
    
    # Set the rate of time update to 1Hz
    rate = rospy.Rate(100)
    
    # Initialize the timer
    timer = 0.0
    
    # Time increment per tick, based on the update rate (5Hz => 0.2 seconds per update)
    time_increment = 0.03
    
    while not rospy.is_shutdown():
        # Publish the current value of the timer
        timer_pub.publish(timer)
        
        rospy.loginfo("Timer: %s seconds" % timer)
        
        # Increment the timer
        timer += time_increment
        
        # Sleep to maintain the 5Hz update rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_timer()
    except rospy.ROSInterruptException:
        pass
