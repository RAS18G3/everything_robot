#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from pynput import keyboard
import actionlib
from actoinclient import CommState
import tf2_ros
from er_planning.msg import PathAction, PathActionGoal
from nav_msgs.srv import GetPlan
from std_srvs.srv import Trigger
from er_perception.msg import ObjectList
import random
import time



class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.path_client = actionlib.SimpleActionClient('path', PathAction)

        self.object_subscriber = rospy.Subscriber("/objects", ObjectList, self.objects_cb)
        self.twist_publisher = rospy.Publisher("/cartesian_motor_controller/twist", Twist, queue_size=1)

    def objects_cb(self, object_list):
        self.objects = object_list.objects

    def run(self):
        command = ''
        while command != 'quit' and command != 'exit':
            command = raw_input('# ')
            snippets = command.split(' ')
            if snippets[0] == 'goto':
                try:
                    self.goto(float(snippets[1]), float(snippets[2]))
                except:
                    print('Wrong arguments...')
            if snippets[0] == 'explore':
                self.explore()
            if snippets[0] == 'grab':
                try:
                    self.grab(int(snippets[1]))
                except:
                    print('Wrong arguments...')
                    print('Available objects are: ')
                    print(self.objects)
            if snippets[0] == 'retrieve':
                try:
                    self.retrieve(int(snippets[1]))
                except:
                    print('Wrong arguments...')
                    print('Available objects are: ')
                    print(self.objects)

            elif command == 'quit' or command == 'exit':
                pass
            else:
                print('Unknown command')

    def retrieve(self, id):
        if self.grab(id):
            if self.goto(0.2, 0.2):
                self.end_grip()
            else:
                print('Error while going back')
                return False
        else:
            print('Error while gripping')
            return False

    def explore(self):
        start_time = time.time()
        elapsed_time = time.time()-start_time
        while elapsed_time < 30:
            x = random.uniform(0.2, 2.0)
            y = random.uniform(0.2, 2.0)
            print("x:"+str(x)+" y:"+str(y))
            self.goto(x, y)
            elapsed_time = time.time()-start_time
        self.goto(0.2, 0.2)


    def grab(self ,id):
        for object in self.objects:
            if object.id == id:
                success = self.goto(object.x, object.y)
                if success:
                    success = self.grip()
                else:
                    return False
                return True
        print('Object id not found')
        return False

    def grip(self):
        start_grip = rospy.ServiceProxy('/start_grip', Trigger)
        success = start_grip()
        return success

    def end_grip(self):
        end_grip = rospy.ServiceProxy('/end_grip', Trigger)
        success = end_grip()
        return success

    def back_up(self):
        twist_msg = Twist()
        twist_msg.linear.x = -0.1;
        self.twist_publisher.publish(twist_msg);
        rospy.Duration(3).sleep();

    def get_current_pos(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Error looking up transform')
            return False

        current_x = trans.transform.translation.x
        current_y = trans.transform.translation.y
        return current_x, current_y

    def goto(self, x, y, remaining_replans=5):
        # find current robot position
        if remaining_replans == 0:
            print("Had to replan to often, cancel...")
            return False
        print('Trying to find path from {0},{1} to {2},{3}'.format(current_x, current_y, x, y))

        current_x, current_y = self.get_current_pos()

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
            print(plan)

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
            if self.path_client.get_state() == CommState.PREEMPTING:
                current_x, current_y = self.get_current_pos()
                if ((current_x - x)**2 + (current_y-y)**2)**0.5 <= 0.25:
                    return True
                else:
                    self.back_up()
                    return self.goto(x,y,remaining_replans-1)
            if self.path_client.get_state() == CommState.SUCCEEDED:
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False


if __name__ == '__main__':
    try:
        bn = BrainNode()
        bn.run()
    except rospy.ROSInterruptException:
        pass
