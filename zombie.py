#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import ButtonEvent, BumperEvent, WheelDropEvent

class zombie:
    def __init__(self):
	    '''Initializations and decision initialization list for every possible result.'''
	    self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	    self.msg = Twist()
	    # Binary values (obstacle exists or not exists)
	    self.fill1 = 0
	    self.fill2 = 0
	    self.fill3 = 0
	    self.fill4 = 0
	    self.fill5 = 0
	    self.bumper_state = 'free'
	    self.wheeldrop_state = 'ground'

	    # Average value of each sectionx
	    self.sect_1 = 0.0
	    self.sect_2 = 0.0
	    self.sect_3 = 0.0
	    self.sect_4 = 0.0
	    self.sect_5 = 0.0

	    # Movement
	    self.threshold = 0.8
	    self.max_count = 50
	    turn_left = 1
	    turn_right = -1
	    fwd = 0.1
	
	    self.count = 0

	    self.ang = { # Has a preference to move right
		        0: 0.0,
		        1: turn_left,
		       10: turn_left, 
		       11: turn_left,
		      100: 0.0, #turn_left,
		      101: turn_left,
		      110: turn_left,
		      111: turn_left,
		     1000:  turn_right,
		     1001:  turn_right,
		     1010:  turn_right,
		     1011:  turn_right,
		     1100:  turn_right,
		     1101:  turn_right,
		     1110:  turn_right,
		     1111:  turn_left,
		    10000:  0.0,
		    10001:  0.0,
		    10010:  turn_right,
		    10011:  turn_right,
		    10100:  turn_right,
		    10101:  turn_right,
		    10110:  turn_right,
		    10111:  turn_right,
		    11000:  turn_right,
		    11001:  turn_right,
		    11010:  turn_right,
		    11011:  turn_right,
		    11100:  turn_right,
		    11101:  turn_right,
		    11110:  turn_right, 
		    11111:  turn_right}
	    self.fwd = {
		        0:  fwd,
		        1:  fwd,
		       10:  fwd,
		       11:  fwd,
		      100:  0.0,
		      101:  0.0,
		      110:  0.0,
		      111:  0.0,
		     1000:  fwd,
		     1001:  fwd,
		     1010:  fwd,
		     1011:  fwd,
		     1100:  0.0,
		     1101:  0.0,
		     1110:  0.0,
		     1111:  0.0,
		    10000:  fwd,
		    10001:  fwd,
		    10010:  fwd,
		    10011:  fwd,
		    10100:  0.0,
		    10101:  0.0,
		    10110:  0.0,
		    10111:  0.0,
		    11000:  fwd,
		    11001:  fwd,
		    11010:  0.0,
		    11011:  0.0,
		    11100:  0.0,
		    11101:  0.0,
		    11110:  0.0, 
		    11111:  0.0,
		    }
	    self.dbgmsg = { #Always has a preference to move right
		    000:'Move forward',
		    001:'Veer right',
		    010:'Veer right',
		    011:'Veer right',
		    100:'Veer left',
		    101:'Veer left',
		    110:'Veer left',
		    111:'Veer right'}

    def reset_sect(self):
	    '''Resets the below variables before each new scan message is read'''
	    self.sect_1 = 0.0
	    self.sect_2 = 0.0
	    self.sect_3 = 0.0
	    self.sect_4 = 0.0
	    self.sect_5 = 0.0

    def sort(self, laserscan):
	    '''Determines if the distances are close enough.'''
	    entries = len(laserscan.ranges)
	    entry_per_sect = entries / 5.0
	    #for entry in range(0,entries):
	    for i, entry in enumerate(laserscan.ranges):
		    entry_i = entry if (not math.isnan(entry)) else 0.0
		    self.sect_1 += entry_i if (0 < i < entries/5) else 0 
		    self.sect_2 += entry_i if (entries/5 < i < 2*entries/5) else 0
		    self.sect_3 += entry_i if (2*entries/5 < i < 3*entries/5) else 0
		    self.sect_4 += entry_i if (3*entries/5 < i < 4*entries/5) else 0
		    self.sect_5 += entry_i if (4*entries/5 < i < entries) else 0
	    self.sect_1 = float(self.sect_1/entry_per_sect)
	    self.sect_2 = float(self.sect_2/entry_per_sect)
	    self.sect_3 = float(self.sect_3/entry_per_sect)
	    self.sect_4 = float(self.sect_4/entry_per_sect)
	    self.sect_5 = float(self.sect_5/entry_per_sect)
	    rospy.loginfo("sort complete,sect_1: " + '{0:.2f}'.format(self.sect_1) + " sect_2: " + '{0:.2f}'.format(self.sect_2) + " sect_3: " + '{0:.2f}'.format(self.sect_3) + " sect_4: " + '{0:.2f}'.format(self.sect_4) + " sect_5:" + '{0:.2f}'.format(self.sect_5))

    def movement(self):
	    '''Decides if an object is in the sector, with threshold.'''
	    self.fill1 = 1 if self.sect_1 < self.threshold else 0
	    self.fill2 = 1 if self.sect_2 < self.threshold else 0
	    self.fill3 = 1 if self.sect_3 < self.threshold else 0
	    self.fill4 = 1 if self.sect_4 < self.threshold else 0
	    self.fill5 = 1 if self.sect_5 < self.threshold else 0
	    rospy.loginfo("Sect = " + str(self.fill1) + str(self.fill2) + str(self.fill3) + str(self.fill4) + str(self.fill5))
	    
	    if (wheeldrop_state == "dropped" or bumper_state == "pressed"):
	        self.msg.angular.z = 0
	        self.msg.linear.x = 0
	    else:
	        self.msg.angular.z = self.ang[sect]
	        self.msg.linear.x = self.fwd[sect]
	        #rospy.loginfo(self.dbgmsg[sect])
	    if (self.count == self.max_count - 1):
	        self.pub.publish(self.msg)
	    self.count = (self.count + 1) % self.max_count
	    self.reset_sect()
 
    def for_callback(self,laserscan):
	    self.sort(laserscan)
	    self.movement()         
     
    def call_back(scanmsg):
        sub_obj.for_callback(scanmsg)

    def listener(self):
        '''Initializes node, creates subscriber, and states callback 
        function.'''
        def callback_ScanEvent(self, data):
	    self.sort(data)
	    # self.movement(self.sect)	
        def callback_BumperEvent(self, data):
            '''If Bumper pressed, wait a little'''        
            if data.state == BumperEvent.RELEASED:
                self.bumper_state = 'free'
            else:
                self.bumper_state = 'pressed'
        def callback_WheelDropEvent(self, data):
            '''If Wheel drop, wait a little.''' 
            # NOTE: dropped when not touching ground
            if (data.state == WheelDropEvent.RAISED):
                self.wheeldrop_state = 'ground'
            else:
                self.wheeldrop_state = 'dropped'
        rospy.init_node('navigation_sensors')
        rospy.loginfo("Subscriber Starting")
        rospy.spin()
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback_BumperEvent)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, callback_WheelDropEvent)
        rospy.Subscriber('/scan', LaserScan, callback_ScanEvent)


if __name__ == "__main__":
    '''Make zombie class and reset node continually.''' 
    sub_obj = zombie()
    sub_obj.listener()