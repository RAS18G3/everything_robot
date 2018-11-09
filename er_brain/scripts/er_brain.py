#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from pynput import keyboard
import actionlib
import tf2_ros
from er_planning.msg import PathAction, PathActionGoal
from nav_msgs.srv import GetPlan



class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.path_client = actionlib.SimpleActionClient('path', PathAction)

    def run(self):
        command = ''
        while command != 'quit' and command != 'exit':
            command = raw_input('# ')
            snippets = command.split(' ')
            if snippets[0] == 'goto':
                # try:
                self.goto(float(snippets[1]), float(snippets[2]))
                # except:
                    # print('Wrong arguments...')
            elif command == 'quit' or command == 'exit':
                pass
            else:
                print('Unknown command')

    def goto(self, x, y):
        # find current robot position
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Error looking up transform')
            return

        current_x = trans.transform.translation.x
        current_y = trans.transform.translation.y

        print('Trying to find path from {0},{1} to {2},{3}'.format(current_x, current_y, x, y))

        rospy.wait_for_service('/pathfinder/find_path')
        try:
            get_plan = rospy.ServiceProxy('/pathfinder/find_path', GetPlan)
            start = PoseStamped()
            start.pose.position.x = current_x
            start.pose.position.y = current_y
            end = PoseStamped()
            end.pose.position.x = x
            end.pose.position.y = y
            plan = get_plan(start, end, 0)
            print('Path found. Sending to path execution...')

            # send plan to path execution
            # Creates the SimpleActionClient, passing the type of the action
            # (FibonacciAction) to the constructor.

            # Waits until the action server has started up and started
            # listening for goals.
            self.path_client.wait_for_server()

            # Creates a goal to send to the action server.
            goal = PathActionGoal()
            goal.goal.Path = plan.plan

            # Sends the goal to the action server.
            self.path_client.send_goal(goal.goal)
            # Waits for the server to finish performing the action.
            print('Waiting for execution to finish...')
            self.path_client.wait_for_result()
            print('Execution has finished.')
            # Prints out the result of executing the action
            print(self.path_client.get_result())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == '__main__':
    try:
        bn = BrainNode()
        bn.run()
    except rospy.ROSInterruptException:
        pass
