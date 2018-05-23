#!/usr/bin/env python

import rospy

# Kobuki includes
from kobuki_msgs.msg import Led

class Turtlebot():
    def __init__(self):
        rospy.init_node('turtlebot_notifier')

        # Preparing leds
        self._led_colors = {
            'off': Led.BLACK,
            'black': Led.BLACK,
            'green': Led.GREEN,
            'orange': Led.ORANGE,
            'red': Led.RED,
        }

        self._led_pubs = {
            '1': rospy.Publisher('/mobile_base/commands/led1', Led, queue_size= 3),
            '2': rospy.Publisher('/mobile_base/commands/led2', Led, queue_size= 3),
        }
        self._led_status = {
            '1': 'green',
            '2': 'green'
        }

        self.Set_led(1, 'off')
        self.Set_led(2, 'off')

        # Preparing spin
        self.spin_time = 1
        self._led_blinker_time = {
            '1': 0.0,
            '2': 0.0
        }

        return

    def Set_led(self, led, color):
        # Set the color of an LED
        #    You can set LED 1 or LED 2 to any of these colors:
        #    - 'off'/'black'
        #    - 'green'
        #    - 'orange'
        #    - 'red'
        #    Example:
        #        robot.set_led(1, 'green')
        #        robot.set_led(1, 'off')
        if str(led) not in self._led_pubs:
            print ("!! Invalid led " + str(led) + ", must be either '1' or '2'")
            return
        if color not in self._led_colors:
            print ("!! Invalid led color " + color)
            return
        if self._led_status[str(led)] != color:
            print (color)
            self._led_pubs[str(led)].publish(Led(self._led_colors[color]))
            self._led_status[str(led)] = color
        return

    def Blinking_led(self, led, color, time):
        # Makes a led blink at the specified time 
        # You can only call this function once per loop
        if str(led) not in self._led_pubs:
            print ("!! Invalid led " + str(led) + ", must be either '1' or '2'")
            return
        if color not in self._led_colors:
            print ("!! Invalid led color " + color)
            return
        self._led_blinker_time[str(led)] += self.spin_time
        if (self._led_blinker_time[str(led)] < time/2.0):
            self.Set_led( led, color)
        else: 
            if (self._led_blinker_time[str(led)] < time):
                self.Set_led( led, 'off')
            else:
                self.Set_led( led, color)
                self._led_blinker_time[str(led)] -= time
            

    def Spin(self):
        # Main loop that spins while the robot moves  
        
        rate = rospy.Rate(1/self.spin_time)
        while not rospy.is_shutdown():
            rospy.loginfo("Hey")
            self.Blinking_led( 1, "green", 5)
            rate.sleep()


turtle = Turtlebot()

turtle.Spin()



# import sys
# import time
# import numpy as np
# import random

# from math import radians



# from kobuki_msgs.msg import BumperEvent

# from kobuki_msgs.msg import Sound
# from kobuki_msgs.msg import WheelDropEvent
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from tf import transformations as trans

# _turtlebot_singleton = None



# class Turtlebot(object):
#     max_linear = 1.0
#     max_angular = 2.0

#         self.__sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)



#     sounds = {
#         'turn on': Sound.ON,
#         'turn off': Sound.OFF,
#         'recharge start': Sound.RECHARGE,
#         'press button': Sound.BUTTON,
#         'error sound': Sound.ERROR,
#         'start cleaning': Sound.CLEANINGSTART,
#         'cleaning end': Sound.CLEANINGEND,
#     }

#     def play_sound(self, sound_type):
#         """Plays a sound on the Turtlebot
#         The available sound sequences:
#             - 0 'turn on'
#             - 1 'turn off'
#             - 2 'recharge start'
#             - 3 'press button'
#             - 4 'error sound'
#             - 5 'start cleaning'
#             - 6 'cleaning end'
#         You can either pass the string or number above
#         """
#         if not isinstance(sound_type, (int, str)):
#                 self.say("!! Invalid sound type, must be an Integer or a String!")
#                 return
#         if isinstance(sound_type, str):
#             try:
#                 sound_type = self.sounds[sound_type]
#             except KeyError:
#                 self.say("!! Invalid sound '{0}', must be one of: {1}"
#                          .format(sound_type, self.sounds.keys()))
#                 return
#         self.__sound_pub.publish(Sound(sound_type))



#     def get_ranges(self):
#         return self.current_laser_msg.ranges

#     def say_ranges(self):
# 	msg = self.current_laser_msg
# 	rngs = msg.ranges
# 	self.say("range angle_min is {0}".format(msg.angle_min))
# 	self.say("range angle_max is {0}".format(msg.angle_max))
# 	self.say("range size is {0}".format(len(rngs)))
# 	self.say("range min is {0}".format(msg.range_min))
# 	self.say("range max is {0}".format(msg.range_max))

# 	for i in range(0, 63):
# 	    rng_pos = i * 10
# 	    self.say("Range {0} is {1}".format(rng_pos, rngs[rng_pos]))
# 	    rng_pos += 5
# 	    self.say("Range {0} is {1}".format(rng_pos, rngs[rng_pos]))

#     def index_to_rad(self, idx):
#         msg = self.current_laser_msg
#         rng = msg.angle_max
#         rlen = len(msg.ranges)
#         self.say("rng: " + str(rng))
#         self.say("rlen: " + str(rlen))
#         return -(rlen / 2.0 - idx) * rng / (rlen / 2.0)
    
#     def turn_around(self):
#         self.turn_angle(np.pi)

#     def random_angle(self):
#         return random.uniform(-np.pi,np.pi)

#     def turn_random(self):
#         self.turn_angle(self.random_angle())

#     def find_closest(self):
#         msg = self.current_laser_msg
#         rngs = msg.ranges
#         idx = np.nanargmin(rngs)
#         self.say("idx: " + str(idx))                
#         rad = self.index_to_rad(idx)
#         return rngs[idx], rad

#     def point_at_closest(self):
# 	rng, rad = self.find_closest()
# 	self.turn_angle(rad)

#     def reset_movement(self):
#         self.movement_enabled = True

#     def show_laser(self):
#         from IPython import display
#         from pylab import subplot, show

#         lm = self.current_laser_msg
#         r = lm.ranges
#         theta = [radians(90) + lm.angle_min + i * lm.angle_increment for i, x in enumerate(r)]

#         ax = subplot(111, polar=True)
#         ax.plot(theta, r, color='r', linewidth=3)
#         ax.set_rmax(6)
#         ax.grid(True)

#         ax.set_title("A line plot of the laser data", va='bottom')
#         show()

#     def __odom_handler(self, msg):
#         self.__x = msg.pose.pose.position.x
#         self.__y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         a = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

#         # cumulative angle doesn't wrap. assumes we've not moved more than pi radians
#         # since last odom message
#         if self.__have_odom:
#             a_diff = a - self.__angle
#             if a_diff > np.pi:
#                 a_diff -= 2*np.pi
#             elif a_diff < -np.pi:
#                 a_diff += 2*np.pi
#             self.__cumulative_angle += a_diff

#         self.__angle = a
#         self.__have_odom = True

#     def __scan_handler(self, msg):
#         self.current_laser_msg = msg

#     def __bumper_handler(self, msg):
#         if msg.state != BumperEvent.PRESSED:
#             return
#         if msg.bumper not in [BumperEvent.CENTER, BumperEvent.LEFT, BumperEvent.RIGHT]:
#             return
#         if self.on_bumper is not None:
#             self.on_bumper.__call__()

#     def __exit_if_movement_disabled(self):
#         if not self.movement_enabled:
#             self.say("Movement currently disabled")
#             sys.exit()

#     def __wheeldrop_handler(self, msg):
#         if msg.state == WheelDropEvent.DROPPED:
# self.movement_enabled = False