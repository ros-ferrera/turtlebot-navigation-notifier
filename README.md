# turtlebot_navigation_notifier
Simple node that uses the leds and the speker of the turtlebot 2 to notify the status of the navigation stack.

* The node bips when the navigation stack is fully awake
* In priority order does the following:
  - Navigation Stack PENDING    (The goal has yet to be processed by the action server)
    Sets one led to the orange color
  - Navigation Stack ACTIVE     (The goal is currently being processed by the action server)
    Leds blinking in orange, in an alternate way. Your robot should be moving.
  - Navigation Stack PREEMPTED  (The goal received a cancel request, terminal State)
    Leds blinking in green, in an alternate way.
  - Navigation Stack SUCCEEDED  (The goal was achieved successfully by the action server, terminal State)
    Sets one led to green, plays a small bip. Your robot reached your destination
  - Navigation Stack ABORTED    (The goal was aborted, terminal state)
    Leds blinking in red, in an alternate way. Small bip. Your robot did not reached its destination.
* Any other state (REJECTED, PREEMPTING, RECALLING, RECALLED and LOST) is ignored by the system

# Requirements
* Kobuki base (base from a turtlebot 2)
* Navigation stack

# Guidelines
To run the node execute:
* rosrun turtlebot_navigation_notifier notifier_node.py
