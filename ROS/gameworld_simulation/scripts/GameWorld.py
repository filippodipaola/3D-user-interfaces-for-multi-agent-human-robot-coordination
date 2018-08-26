#!/usr/bin/env python

import numpy as np
from TaskQueue import TaskQueue
import random as rnd
from Agents import ConstructionAgent, TransportAgent
from Tasks import ConstructionTask, MoveTask, TransportTask
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import math
from ROSController import ROSController
import copy
from Timer import Timer
import sys
class GameWorld():

    def __init__(self, size=(100, 100), agents=[], tasks=TaskQueue(), AI_ON=True):
        self.size = size
        self.agents = agents
        self.tasks = tasks
        self.map = np.ones(size)
        self.user_priority_queue = TaskQueue()
        self.world_task_queue = TaskQueue()
        self.generate_agents()
        self.generate_world_tasks()
        #self.assign_tasks()
        self.ros_controller = ROSController()
        self.restart = False
        #self.agents[0].task_queue.prioritise(self.world_task_queue.get_queue()[0])


    def update(self):

        # Prints the tasks in each of the agent queues for debugging reasons.
        #for agent in self.agents:
        #    print ("Agent: %s has the tasks: %s" % (agent.name, ",".join([t.name for t in agent.task_queue.get_queue()])))

        if not self.world_task_queue.is_empty():
            #print("Publishing tasks")
            self.ros_controller.publish_tasks(self.world_task_queue.get_queue())
        #print("Publishing Agents")
        self.ros_controller.publish_agents(self.agents)

        ## This checks if the game is meant to run yet
        if self.ros_controller.game_start:
            self.check_scenario_complete()
            for agent in self.agents:
                agent.do_work()

        if self.ros_controller.recieved_assign_message:
            #print("I SHOULDN'T HAVE A ASSIGN MESSAGE YET: %s" % self.ros_controller.recieved_assign_message)
            self.ros_controller.recieved_assign_message = False
            assigned_agent = self.find_agent_in_list(self.ros_controller.user_assign_task[0])
            assigned_task = self.find_task_in_queue(self.ros_controller.user_assign_task[1])
            assigned_agent.add_task(assigned_task)
            print("Assigned agent %s to do task %s" % (assigned_agent.name, assigned_task.name))


        if self.ros_controller.recieved_move_message:
            self.ros_controller.recieved_move_message = False
            assigned_agent = self.find_agent_in_list(self.ros_controller.user_assign_move[0])
            pose = self.pose_constructor(float(self.ros_controller.user_assign_move[1][0]),
                                         float(self.ros_controller.user_assign_move[1][1]),0,0,0,0,1)
            assigned_agent.set_current_task(MoveTask("TempMoveTask", "Agent needs to move to this spot", end_pose=pose, agent=assigned_agent))
            print("Assigned agent %s to move to %s" %(assigned_agent.name, pose))
        # DEBUG STUFF!
        # print("!!!!!!CURRENT LOCATION OF AGENTS AND THEIR TARGETS!!!!!!")
        # for agent in self.agents:
        #
        #     print("AGENT: %s - Position:\n %s \n TASK: %s - Position:\n %s" % (agent.name,
        #                                                                 agent.pose.position,
        #                                                                 agent.task_queue.peek().name,
        #                                                                 agent.task_queue.peek().pose.position))
        # print("---------------------------------------------------------")

    def find_task_in_queue(self, task_name):
        for task in self.world_task_queue.get_queue():
            if task.name == task_name:
                return task

        print("Could not find the task..")
        return None

    def find_agent_in_list(self, agent_name):
        for agent in self.agents:
            print("ListAgent: %s == InputAgent: %s = %s" % (agent.name, agent_name, agent.name == agent_name) )
            if agent.name == agent_name:
                return agent
        print [ agent.name for agent in self.agents]
        print "Could not find the agent!"
        return None

    def check_scenario_complete(self):
        for t in self.world_task_queue.get_queue():
            if not t.is_complete:
                return False

        timer = Timer()
        while not self.ros_controller.got_end_confirmation:
            message = String()
            message.data = "finished"
            self.ros_controller.publish_update_message(message)
            message.data = "end time|%s" %timer.get_elasped_time()
            self.ros_controller.publish_update_message(message)
        print("All assigned tasks have been completed!")
        self.restart = True


    def add_agent(self, agent):
        self.agents.append(agent)

    def add_task(self, task):
        self.tasks.enqueue(task)
        self.world_task_queue.enqueue(task)

    def get_nearest_tasks(self, agent):
        nearest_task = None
        # Finds the furthest possible distance between two points of a rectangluar grid.
        nearest_dist = math.sqrt(math.pow(self.size[0], 2) + math.pow(self.size[1], 2))
        for task in agent.task_queue.get_queue():
            if not task.assigned_agent:
                dist = math.sqrt(math.pow(task.pose.position.x - agent.pose.position.x, 2)
                                 + math.pow(task.pose.position.y - agent.pose.position.y, 2)
                                 + math.pow(task.pose.position.z - agent.pose.position.z, 2))
                if dist < nearest_dist:
                    nearest_task = task
        # Ensures no "current task" is assigned to multiple agents,
        #self.tasks.remove(nearest_task)
        task.assigned_agent = agent
        return nearest_task

    def get_compabitible_task_for_agent(self, agent):
        compatible_task_list = TaskQueue()
        for task in self.tasks.get_queue():
            if task.is_agent_compatible(agent):
                compatible_task_list.enqueue(task)


        if compatible_task_list.is_empty():
            print "No compatible tasks found for agent %s" %agent.name
            return None

        return compatible_task_list



    def assign_tasks(self):
        for agent in self.agents:
            agent.task_queue = self.get_compabitible_task_for_agent(agent)
            if agent.task_queue:
                agent.set_current_task(self.get_nearest_tasks(agent))


    def generate_agents(self, num_of_agents=0):
        pose1 = Pose()
        pose1.position.x = 0
        self.add_agent(ConstructionAgent('Construction Agent 1', self.pose_constructor(60, 50, 0, 0, 0, 0, 1)))
        self.add_agent(ConstructionAgent('Construction Agent 2', self.pose_constructor(40, 50, 0, 0, 0, 0, 1)))
        self.add_agent(ConstructionAgent('Construction Agent 3', self.pose_constructor(50, 40, 0, 0, 0, 0, 1)))
        self.add_agent(TransportAgent('Transport Agent 1', pose=self.pose_constructor(50, 60, 0, 0, 0, 0, 1)))

    def pose_constructor(self, posX, posY, posZ, quadX, quadY, quadZ, quadW):
        tempPose = Pose()
        tempPose.position.x = posX
        tempPose.position.y = posY
        tempPose.position.z = posZ
        tempPose.orientation.x = quadX
        tempPose.orientation.y = quadY
        tempPose.orientation.z = quadZ
        tempPose.orientation.w = quadW
        return tempPose

    def generate_world_tasks(self, no_of_task=0):
        self.add_task(ConstructionTask('Construction Task 1',"Construction required in this area",
                                       pose=self.pose_constructor(50, 10, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 2', "Construction required in this area",
                                       pose=self.pose_constructor(60, 20, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 3', "Construction required in this area",
                                       pose=self.pose_constructor(40, 27, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 4', "Construction required in this area",
                                       pose=self.pose_constructor(10, 65, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 5', "Construction required in this area",
                                       pose=self.pose_constructor(65, 40, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 6', "Construction required in this area",
                                       pose=self.pose_constructor(15, 75, 0, 0, 0, 0, 1)))
        self.add_task(TransportTask('Transport Task 1', "Transport required",
                                       pose=self.pose_constructor(45, 65, 0, 0, 0, 0, 1), end_pose=self.pose_constructor(10, 80, 0, 0, 0, 0, 1)))





