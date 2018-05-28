#!/usr/bin/env python
#
# MIT License

# Copyright (c) 2018 ros-ferrera

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy

# Kobuki includes
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

# Navigation scatk includes
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus

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
            self.__led_pub = rospy.Publisher('mobile_base/commands/led' + str(led_num), Led, queue_size= 3)
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
        self.__blink_options = {
            'on':  True,
            'off': False
        }
        self.__blink_color = 'orange'

        self.__Set_led('off')
        #print("Turtle led "+ str(led_num) +" prepared")

        self.__timer = []

        return

    def Status(self, color, status = "off", start_blinking_in = "on", time = "1"):
        # Change the status of a led (if is blinking, it can start_blinking_in on or off)

        if color not in self.__led_colors:
            print ("!! Invalid led color " + color)
            rospy.signal_shutdown("!! Invalid led color " + color)
            return
        if status not in self.__possible_status:
            print ("!! Invalid led status " + status)
            rospy.signal_shutdown("!! Invalid led status " + status)
            return
        if time <= 0.0:
            print ("!! Invalid blinking time " + str(time))
            rospy.signal_shutdown("!! Invalid blinking time" + str(time))
            return
        if start_blinking_in not in self.__blink_options:
            print ("!! Invalid blinking status " + start_blinking_in)
            rospy.signal_shutdown("!! Invalid blinking status " + start_blinking_in)
            return

        # Resetting any old timer
        if self.__timer:
            self.__timer.shutdown()

        if status == "blinking":
            self.__blink_color = color
            self.__timer = rospy.Timer(rospy.Duration(int(time)), self.__Blink)
            if self.__blink_options[start_blinking_in]:
                color = "off"

        self.__Set_led(color)
        self.__led_status = status

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
            #print ("Led" + str(self.__led_num) + ": " + color)
            self.__led_pub.publish(Led(self.__led_colors[color]))
            self.__color_status = color
        return
    
class TurtleSound():
    # Class to control the sounds of your turtlebot 2.
    # Use the method .Play with the following options:
    #   - 0 'turn on'
    #   - 1 'turn off'
    #   - 2 'recharge start'
    #   - 3 'press button'
    #   - 4 'error sound'
    #   - 5 'start cleaning'
    #   - 6 'cleaning end'
    def __init__(self):
        
        self.__sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size= 3)

        self.__sounds = {
            'turn on': Sound.ON,
            'turn off': Sound.OFF,
            'recharge start': Sound.RECHARGE,
            'press button': Sound.BUTTON,
            'error sound': Sound.ERROR,
            'start cleaning': Sound.CLEANINGSTART,
            'cleaning end': Sound.CLEANINGEND,
        }

    def Play(self, sound):
        # Plays a sound on the Turtlebot
        # The available sound sequences:
        #    - 0 'turn on'
        #    - 1 'turn off'
        #    - 2 'recharge start'
        #    - 3 'press button'
        #    - 4 'error sound'
        #    - 5 'start cleaning'
        #    - 6 'cleaning end'
        #You can either pass the string or number above
        if not isinstance(sound, (int, str)):
                self.say("!! Invalid sound type, must be an Integer or a String!")
                return
        if isinstance(sound, str):
            try:
                sound = self.__sounds[sound]
            except KeyError:
                print ("!! Invalid sound")
                return
        self.__sound_pub.publish(Sound(sound))


class Turtlebot():
    def __init__(self):
        rospy.init_node('turtlebot_notifier')

        # Preparing leds
        self.__leds = {
            '1':  TurtleLed(1),
            '2':  TurtleLed(2)
        }

        # Preparing sounds
        self.__sounds = TurtleSound()

        # The system requires some time to prepare themselves
        rate = rospy.Rate(1)
        rate.sleep()

        self.__leds['1'].Status("green")
        self.__leds['2'].Status("off")

        # Preparing spin
        self.__spin_time = 1
        self.__rate = rospy.Rate(1/self.__spin_time)

        # Preparing the communication with the navigation stack
        self.__nav_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.__NavStack_callback, queue_size= 10)
        self.__last_state = GoalStatus.SUCCEEDED 
        self.__nav_stack_awake = False
        return

    def __NavStack_callback(self,msg):
        # Callback with the information from the navigation stack
        if (not self.__nav_stack_awake):
            self.__nav_stack_awake = True
            # print "Navigaton stack on"
            self.__sounds.Play('turn on')

        if len(msg.status_list) == 0:
            return
        
        
        dominant_status = self.__DominantStatus(msg.status_list)
        if (dominant_status != self.__last_state):
            self.__SetNewStatus(dominant_status)

        # print "-----------"
        # for msg in msg.status_list:
        #     if (msg.status == dominant_status):
        #         string = "d->"
        #     else:
        #         string = "   "
        #     print (string + str(msg.status) + ":" + msg.text)
        # print "-----------"
        return

    def __DominantStatus(self, all_status):
        # Returns the dominant status in a list of status
        status_list = []
        for msg in all_status:
            status_list.append(msg.status)
            
        return min(status_list)
        
    def __SetNewStatus(self, status):
        # Shows a new status with the leds and sounds of the turtlebot
        self.__last_state = status 
        
        #http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        if (status == GoalStatus.PENDING):
            # print ("pending")
            self.__leds['2'].Status("orange")
            return
        if (status == GoalStatus.PREEMPTED): # 2
            # print ("preempted")
            # Transition status (reset leds) (silent active)
            self.__leds['1'].Status("green", "blinking", "on")
            self.__leds['2'].Status("green", "blinking", "off")
            return
        if (status == GoalStatus.ACTIVE): # 1
            # print ("active")
            # The goal is currently being processed by the action server
            self.__leds['1'].Status("orange", "blinking", "on")
            self.__leds['2'].Status("orange", "blinking", "off")
            self.__sounds.Play(3) # Small bip
            return
        if (status == GoalStatus.SUCCEEDED):
            # print ("succeed")
            self.__leds['1'].Status("green")
            self.__leds['2'].Status("off")
            self.__sounds.Play(3) # Small bip
            return
        if (status == GoalStatus.ABORTED):
            # print ("aborted")
            self.__leds['1'].Status("red","blinking", "on")
            self.__leds['2'].Status("red","blinking", "off")
            self.__sounds.Play(4)  #Complaining
            return
        # if (status == GoalStatus.REJECTED):
        #     print ("rejected")
        #     return
        # if (status == GoalStatus.PREEMPTING):
        #     print ("preempting")
        #     return
        # if (status == GoalStatus.RECALLING):
        #     print ("recalling")
        #     return
        # if (status == GoalStatus.RECALLED):
        #     print ("recalled")
        #     return
        # if (status == GoalStatus.LOST):
        #     print ("lost")
        #     return
        
    def Spin(self):
        # Main loop that spins while the robot moves  
        
        while not rospy.is_shutdown():
            self.__rate.sleep()


turtle = Turtlebot()

turtle.Spin()
