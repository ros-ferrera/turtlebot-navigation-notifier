#!/usr/bin/env python

import rospy

# Kobuki includes
from kobuki_msgs.msg import Led

class TurtleLed():
    # Controls the status of one of the leds of the turtlebot2
    # To update it, use .update() method
    # To change its status, use .status(color, status, time) method
    #   colors = "off", "black", "green", "orange", "red"
    #   status = "off", "static", "blinking"
    #   time   = blinking time
    def __init__(self, led_num):
        if (led_num == 1 or led_num == 2):
            self.__led_num = led_num
            self.__led_pub = rospy.Publisher('/mobile_base/commands/led' + str(led_num), Led, queue_size= 3)
        else:
            print ("!! Invalid led " + str(led) + ", must be either '1' or '2'")
            rospy.signal_shutdown("Wrong led number used")
        
        # Preparing possible status
        self.__possible_status = {
            'off', 'static', 'blinking'
        }
        self.__led_status   = 'static'

        self.__led_colors = {
            'off': Led.BLACK,
            'black': Led.BLACK,
            'green': Led.GREEN,
            'orange': Led.ORANGE,
            'red': Led.RED,
        }
        self.__color_status = 'other'

        # Blinker control
        self.__blink_color = 'orange'

        self.__Set_led('off')
        print("Turtle led "+ str(led_num) +" prepared")

        self.__timer = []

        return

    def Status(self, color, status = "off", time = "1"):
        # Change the status of a led

        if color not in self.__led_colors:
            print ("!! Invalid led color " + color)
            rospy.signal_shutdown("!! Invalid led color " + color)
            return
        if status not in self.__possible_status:
            print ("!! Invalid led status " + status)
            rospy.signal_shutdown("!! Invalid led color " + status)
            return
        if time <= 0.0:
            print ("!! Invalid blinking time " + str(time))
            rospy.signal_shutdown("!! Invalid blinking time" + str(time))
            return

        self.__Set_led(color)
        self.__led_status = status

        if status == "blinking":
            self.__blink_color = color
            self.__timer = rospy.Timer(rospy.Duration(int(time)), self.__Blink)
        else:
            if self.__timer:
                self.__timer.shutdown()

        return

    def __Blink(self, event):
        # Makes the led blink

        if self.__color_status != 'off':
            self.__Set_led('off')
        else: 
            self.__Set_led(self.__blink_color)

        return

    def __Set_led(self, color):
        # Set the color of an LED
        #    You can set LED to any of these colors:
        #    - 'off'/'black'
        #    - 'green'
        #    - 'orange'
        #    - 'red'
        #    Example:
        #        robot.set_led('green')
        #        robot.set_led('off')
        if color not in self.__led_colors:
            print ("!! Invalid led color " + color)
            rospy.signal_shutdown("!! Invalid led color " + color)
            return
        if self.__color_status != color:
            print ("Led" + str(self.__led_num) + ": " + color)
            self.__led_pub.publish(Led(self.__led_colors[color]))
            self.__color_status = color
        return
    
class Turtlebot():
    def __init__(self):
        rospy.init_node('turtlebot_notifier')

        # Preparing leds
        self.__leds = {
            '1':  TurtleLed(1),
            '2':  TurtleLed(2)
        }

        # The system requires some time to prepare themselves
        rate = rospy.Rate(1)
        rate.sleep()

        self.__leds['1'].Status("off")
        self.__leds['2'].Status("off")

        # Preparing spin
        self.spin_time = 1

        return

    def Spin(self):
        # Main loop that spins while the robot moves  
        
        rate = rospy.Rate(1/self.spin_time)
        while not rospy.is_shutdown():
            rospy.loginfo("Hey")
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







# Example code for the navigation stack
# import rospy

# # Brings in the SimpleActionClient
# import actionlib
# # Brings in the .action file and messages used by the move base action
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# def movebase_client():

#    # Create an action client called "move_base" with action definition file "MoveBaseAction"
#     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
#    # Waits until the action server has started up and started listening for goals.
#     client.wait_for_server()

#    # Creates a new goal with the MoveBaseGoal constructor
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#    # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
#     goal.target_pose.pose.position.x = 0.5
#    # No rotation of the mobile base frame w.r.t. map frame
#     goal.target_pose.pose.orientation.w = 1.0

#    # Sends the goal to the action server.
#     client.send_goal(goal)
#    # Waits for the server to finish performing the action.
#     wait = client.wait_for_result()
#    # If the result doesn't arrive, assume the Server is not available
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#     # Result of executing the action
#         return client.get_result()   

# # If the python node is executed as main process (sourced directly)
# if __name__ == '__main__':
#     try:
#        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
#         rospy.init_node('movebase_client_py')
#         result = movebase_client()
#         if result:
#             rospy.loginfo("Goal execution done!")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")