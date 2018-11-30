#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from pynput import keyboard
import actionlib
from actionlib import CommState
import tf2_ros
from er_planning.msg import PathAction, PathActionGoal
from nav_msgs.srv import GetPlan
from std_srvs.srv import Trigger
from er_perception.srv import RemoveObject
from er_perception.msg import ObjectList
from nav_msgs.msg import OccupancyGrid
import random
import time
import numpy as np

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.path_client = actionlib.SimpleActionClient('path', PathAction)

        # default values of each object:
        self.object_values = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,0]

        self.object_subscriber = rospy.Subscriber("/objects", ObjectList, self.objects_cb)
        self.map_subscriber = rospy.Subscriber("/slam/occupancy_grid", OccupancyGrid, self.map_cb)
        self.twist_publisher = rospy.Publisher("/cartesian_motor_controller/twist", Twist, queue_size=1)

    def objects_cb(self, object_list):
        self.objects = object_list.objects

    def map_cb(self, OccupancyGrid):
        self.map = OccupancyGrid

    def run(self):
        command = ''
        reset_map = rospy.ServiceProxy('/slam/reset_localization', Trigger)
        reset_map()
        print("Reset map. Brain ready.\n")

        while command != 'quit' and command != 'exit':
            command = raw_input('# ')
            snippets = command.split(' ')

            if snippets[0] == 'goto':
                self.goto(float(snippets[1]), float(snippets[2]))
                try:
                    pass
                except (ValueError, IndexError) as e:
                    print('Wrong arguments...')
                    print(snippets[1])
                    print(snippets[2])

            elif snippets[0] == 'explore':
                self.explore()

            elif snippets[0] == 'grab':
                try:
                    self.grab(int(snippets[1]))
                except (ValueError, IndexError) as e:
                    print('Wrong arguments...')
                    print('Available objects are: ')
                    LABELS = ['Yellow Ball', 'Yellow Cube', 'Green Cube', 'Green Cylinder', 'Green Hollow Cube', 'Orange Cross', 'Patric', 'Red Cylinder', 'Red Hollow Cube', 'Red Ball', 'Blue Cube', 'Blue Triangle', 'Purple Cross', 'Purple Star', 'Other']
                    for object in self.objects:
                        print("id: " + str(object.id)+", type: " + LABELS[object.class_id] + ", value: " + str(self.object_values[object.class_id]))
                        print("pos: ["+str(object.x)+", "+str(object.y)+"]\n")

            elif snippets[0] == 'retrieve':
                if snippets[1] == 'best':
                    self.retrieve_best()
                elif snippets[1] == 'all':
                    self.retrieve_all()
                else:
                    try:
                        self.retrieve(int(snippets[1]))
                    except (ValueError, IndexError) as e:
                        print('Wrong arguments...')
                        print('Available objects are: ')

            elif snippets[0] == 'clear':
                if len(snippets) < 2 or snippets[1] == 'all':
                    self.clear(-1)
                else:
                    self.clear(int(snippets[1]))

            elif command == 'quit' or command == 'exit':
                pass
            else:
                print('Unknown command')

    def clear(self, id):
        delete_object = rospy.ServiceProxy('/object_filter_node/remove_object', RemoveObject)
        if id == -1:
            for object in self.objects:
                    delete_object(object.id)
            print("Deleted all objects")
        else:
            try:
                delete_object(id)
                print("Deleted object with id "+str(id))
            except:
                print("I would like you to know that this probably was an invalid id, sir/madam.")

    def retrieve(self, id):
        if self.grab(id):
            if self.goto(0.2, 0.2):
                self.end_grip()
                self.clear(id)
            else:
                print('Error while going back')
                return False
        else:
            print('Error while gripping')
            return False

    def explore(self):
        x_cells = 6
        y_cells = 10
        points_per_cell = 1
        width = self.map.info.width*self.map.info.resolution + 2*self.map.info.origin.position.x
        height = self.map.info.height*self.map.info.resolution + 2*self.map.info.origin.position.y

        width_cell = width/x_cells
        height_cell = height/y_cells
        i = 0
        j = 0
        k = 0
        start_time = time.time()
        elapsed_time = time.time()-start_time
        print("reset map")
        #reset_map = rospy.ServiceProxy('/slam/reset_localization', Trigger);
        #while elapsed_time < 60:

        loopbreak = False
        for i in range(y_cells):
            if loopbreak:
                break
            for j in range(x_cells):
                if loopbreak:
                    break
                min_x = j*width_cell
                min_y = i*height_cell
                max_x = (j+1)*width_cell
                max_y = (i+1)*height_cell
                for k in range(points_per_cell):
                    if loopbreak:
                        break
                    random.seed() # new random seed
                    x = random.uniform(min_x, max_x)
                    y = random.uniform(min_y, max_y)
                    print("x:"+str(x)+" y:"+str(y))
                    self.goto(x, y)
                    elapsed_time = time.time()-start_time
                    if elapsed_time > 120:
                        print("Exploration time ran out.")
                        loopbreak = True
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
        print('Trying to back up...')
        twist_msg = Twist()
        twist_msg.linear.x = -0.1;
        self.twist_publisher.publish(twist_msg);
        rospy.sleep(1.0);
        twist_msg = Twist()
        twist_msg.linear.x = 0;
        self.twist_publisher.publish(twist_msg);
        rospy.sleep(1.0);

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
            print("Had to replan too often, cancel...")
            return False

        current_x, current_y = self.get_current_pos()

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
            print('Execution action client has finished.')
            # Prints out the result of executing the action
            print(self.path_client.get_state())
            if self.path_client.get_state() == 2:
                print('Obstacle found during execution...')
                current_x, current_y = self.get_current_pos()
                if ((current_x - x)**2 + (current_y-y)**2)**0.5 <= 0.25:
                    return True
                else:
                    self.back_up()
                    return self.goto(x,y,remaining_replans-1)
            if self.path_client.get_state() == 3:
                print('Path succesfully executed')
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def reevaluate_object(self,object_index,value):
        self.object_values[object_index] = value

    def retrieve_best(self):
        LABELS = ['Yellow Ball', 'Yellow Cube', 'Green Cube', 'Green Cylinder', 'Green Hollow Cube', 'Orange Cross', 'Patric', 'Red Cylinder', 'Red Hollow Cube', 'Red Ball', 'Blue Cube', 'Blue Triangle', 'Purple Cross', 'Purple Star', 'Other']
        object_vals = []
        for val in self.object_values:
            object_vals.append(val)

        while True:
            # find what object type is most valuable
            index_max = np.argmax(object_vals)

            # if the most valuable object has val 0 then we apparently screwed up
            if object_vals[index_max] == 0:
                print('No valid objects to grab.')
                return False
            #print('Current most valuable object type: '+LABELS[index_max])

            # check if we have seen it and retrieve it
            for object in self.objects:
                dist = (object.x-0.2)**2 + (object.y-0.2)**2
                if object.class_id==index_max and dist > 0.05:
                    print('Retrieving '+LABELS[index_max]+", internal ID: "+str(object.id))
                    if self.retrieve(object.id):
                        return True
                    else:
                        print("Cannot retrieve object "+str(object.id)+", "+LABELS[index_max])
            # if not, set the most valuable to 0 and go for the next
            object_vals[index_max] = 0

    def retrieve_all(self):
        retrieval_success = True
        while(retrieval_success):
            retrieval_success = retrieve_best()

if __name__ == '__main__':
    try:
        bn = BrainNode()
        bn.run()
    except rospy.ROSInterruptException:
        pass
