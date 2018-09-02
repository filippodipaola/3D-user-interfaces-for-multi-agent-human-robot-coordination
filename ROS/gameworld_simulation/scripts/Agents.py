#!/usr/bin/env python
from geometry_msgs.msg import Pose
from TaskQueue import TaskQueue
from Tasks import MoveTask
import math
from CapabilitiesEnums import CapabiltiesEnums as capEnum
from tf.transformations import quaternion_from_euler
from ROSController import ROSController
"""
This class is used as a super class to all agents in gameworld.
Its intention is to lightly recreate a real world robot which
could be simulated in Gazebo, however due to machine and time 
limitation is was more sensible to simplify this into a CPU
lighter format. 
"""

# TODO I got to figure out a simple way of object avoidance. May need numpy and some other libraries
# TODO Potentially I could use some A* algorithm, using the gameworld as a map.
PI = 3.14159265359
class Agent():
    def __init__(self, name, pose=Pose(), size=(1.0, 1.0, 1.0), capabilities=[], movement_speed=0.1):
        # Tuple contain the x,y,z coordinates of the agent
        self.pose = pose # Rospy pose data structure, used in Unity to determine space
        self.name = name # Name of this particular agent, needs to be unique
        self.size = size # the x,y,z size of the agent
        self.task_queue = TaskQueue()
        self.capabilities = capabilities
        self.status = ""
        self.current_action = "idle"
        self.movement_speed = movement_speed
        self.is_at_task = False
        self.ros_controller = ROSController()


    def get_pose(self):
        return self.pose

    ## TODO, fill in this with a custom message.
    def get_message(self):
        pass

    def add_task(self, task):
        self.task_queue.enqueue(task)

    def get_task(self):
        return self.task_queue.peek()

    def set_current_task(self, task):
        self.is_at_task = False;
        self.task_queue.prioritise(task)

    def update_status(self):
        if self.task_queue.is_empty():
            self.status = "%s is idle." % self.name
        else:
            task_name = self.task_queue.peek().name
            self.status = "%s is %s task[ %s ] currently." % (self.name, self.current_action, task_name)

    def get_status(self):
        self.update_status()
        return self.status

    def go_to_current_task(self):
        if self.pose.position == self.task_queue.peek().pose.position:
            print("Agent %s is currently at the it's current task!" % self.name)
            self.is_at_task = True
            return True
        self.current_action = "moving to task"
        self.move_towards_target(self.task_queue.peek().pose.position)
        return False

    def move_towards_target(self, target_pos):
        target_diff_x = target_pos.x - self.pose.position.x
        target_diff_y = target_pos.y - self.pose.position.y
        target_diff_z = target_pos.z - self.pose.position.z
        self.pose.position.x += self.get_move_vector(target_diff_x)
        self.pose.position.y += self.get_move_vector(target_diff_y)
        self.pose.position.z += self.get_move_vector(target_diff_z)
        direction_y = math.atan2(self.get_move_vector(target_diff_y),self.get_move_vector(target_diff_x))
        quat = quaternion_from_euler(0, direction_y, 0)
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]

    def get_move_vector(self, target_x):
        if abs(target_x) > self.movement_speed:
            if target_x < 0:
                return -self.movement_speed
            else:
                return self.movement_speed
        return target_x


    def complete_current_task(self):
        if self.task_queue.peek().is_completed():
            print("Current task %s being done by %s has been completed." %(self.task_queue.peek().name, self.name))
            self.ros_controller.publish_string_update_message("%s has been completed by %s" % (self.task_queue.peek().name, self.name))
            self.task_queue.dequeue()
            self.is_at_task = False
            self.current_action = "completed task"
            return True
        print"Current task %s has not yet been completed by agent %s." %(self.task_queue.peek().name, self.name)
        return False

    # OVERRIDE THIS FUNCTION WITH SPECIFIC AGENT FUNCTION
    def do_work(self):
        pass


class ConstructionAgent(Agent):

    def __init__(self, name, pose=Pose(), size=(1.0, 1.0, 1.0),
                 capabilities=[capEnum.construct, capEnum.drive], construction_speed=1.0, movement_speed=0.1):

        Agent.__init__(self, name, pose, size, capabilities, movement_speed)
        self.construction_speed = construction_speed

    def do_work(self):

        if self.task_queue.is_empty():
            if self.current_action != "idle":
                print("Agent %s has no tasks in its queue, therefore it cannot work." % self.name)
                self.current_action = "idle"
            return False

        if self.get_task().is_complete:
            print("Task %s has already been completed by an agent, agent %s moving to next task" % (self.get_task().name, self.name))
            self.task_queue.dequeue()
            self.is_at_task = False
            return True

        if not self.is_at_task:
            if self.current_action != "moving to task":
                print("Agent %s is not at the location of the task, moving to task" % self.name)
            if self.go_to_current_task():
                print("Agent %s has reached the location of task %s" % (self.name, self.get_task().name))
                if isinstance(self.get_task(), MoveTask):
                    self.complete_current_task()
            return False


        if not isinstance(self.get_task(), MoveTask):
            if self.task_queue.peek().constuct(self):
                self.complete_current_task()
                return True

        self.current_action = "constructing"
        return False

class TransportAgent(Agent):

    def __init__(self, name, pose=Pose(), size=(1.0, 1.0, 1.0),
                 capabilities=[capEnum.pickup, capEnum.transport, capEnum.drive], movement_speed=0.1):

        Agent.__init__(self, name, pose, size, capabilities, movement_speed)

    def move_towards_delivery(self):
        if self.pose.position == self.task_queue.peek().end_pose.position:
            print("Agent %s is at the delivery position of it's current task!" % self.name)
            self.get_task().delivered()

            return True
        self.current_action = "moving to deliver package"
        self.move_towards_target(self.task_queue.peek().end_pose.position)
        return False

    def do_work(self):
        if self.task_queue.is_empty():
            if self.current_action != "idle":
                print("Agent %s has no tasks in its queue, therefore it cannot transport anything." % self.name)
                self.current_action = "idle"
            return False
        #if not isinstance(self.get_task(), MoveTask):
            #if self.get_task().is_picked_up and self.get_task().agent_holding != self:
             #   print("Task %s has already been picked up, removing this task from agent %s" % (self.get_task().name, self.name))
              #  self.task_queue.dequeue()
        if not self.is_at_task:
            if self.current_action != "moving to task":
                print("Agent %s is not at the location of the task, moving to task" % self.name)
            if self.go_to_current_task():
                print("Agent %s has reached the location of task %s" % (self.name, self.get_task().name))
                if isinstance(self.get_task(), MoveTask):
                    self.complete_current_task()


            return False

        if not isinstance(self.get_task(), MoveTask):
            if not self.task_queue.peek().is_picked_up:
                self.task_queue.peek().pick_up(self)
                print("AGENT PICKED UP DILIVERY!! WHY !")
                return False
            if self.move_towards_delivery():
                print("AGENT COMPLETED DILIVERY, THIS TASK SHOULD NO LONGER EXIST.")
                self.complete_current_task()
                return True
            else:
                self.get_task().update_position(self.pose.position)





        if self.move_towards_target(self.task_queue.peek().end_pose.position):
            self.task_queue.peek().delivered()
            self.complete_current_task()
            return True
        else:
            self.task_queue.peek().update_position(self.pose.position)
            return False

        self.current_action = "transporting"
        return False




