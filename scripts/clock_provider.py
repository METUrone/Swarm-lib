#!/usr/bin/python3    

"""
Publishes time data to /clock topic in order to explicitly give simulation time
"""                                                                             

import time
import rospy
from std_msgs.msg import Time,Duration
from rosgraph_msgs.msg import Clock

if __name__ == '__main__':

    rospy.init_node('sim_clock', anonymous = True)
    clock_pub = rospy.Publisher("/clock", Clock, queue_size = 10)
    clock = Clock()
    
    # Publish frequency
    clock_rate = rospy.get_param('/clock_frequency')
    # Sets the value of the published time
    real_time_factor = rospy.get_param('/real_time_factor')

    clock.clock = rospy.Time.from_sec(1.0 / clock_rate)
    
    duration = rospy.Duration.from_sec(1.0 / clock_rate)

    
    sleep_duration = 1.0 / (clock_rate * real_time_factor)

    while not rospy.is_shutdown():
        t1 = time.time()
        clock.clock = clock.clock + duration
        # publishes at a rate given by ~clock_frequency*real_time_factor
        clock_pub.publish(clock)
        time.sleep(max(0, sleep_duration  - (time.time() - t1)))
        
