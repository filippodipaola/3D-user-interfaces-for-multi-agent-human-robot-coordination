#!/usr/bin/env python

import numpy as np
from TaskQueue import TaskQueue
import random as rnd
from Agents import ConstructionAgent
from Tasks import ConstructionTask
from geometry_msgs.msg import Pose
import math
from ROSController import ROSController
import copy
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
        self.assign_tasks()
        self.world_task_queue = copy.copy(tasks)
        self.ros_controller = ROSController()

    def update(self):
        self.check_scenario_complete()
        for agent in self.agents:
            agent.do_work()
            self.ros_controller.publish_agents(self.agents)
            if not self.tasks.is_empty():
                self.ros_controller.publish_tasks(self.tasks.get_queue())
        # DEBUG STUFF!
        # print("!!!!!!CURRENT LOCATION OF AGENTS AND THEIR TARGETS!!!!!!")
        # for agent in self.agents:
        #
        #     print("AGENT: %s - Position:\n %s \n TASK: %s - Position:\n %s" % (agent.name,
        #                                                                 agent.pose.position,
        #                                                                 agent.task_queue.peek().name,
        #                                                                 agent.task_queue.peek().pose.position))
        # print("---------------------------------------------------------")

    def check_scenario_complete(self):
        for t in self.world_task_queue.get_queue():
            if not t.is_complete:
                return False

        print("All assigned tasks have been completed!")
        sys.exit(1)


    def add_agent(self, agent):
        self.agents.append(agent)

    def add_task(self, task):
        self.tasks.enqueue(task)

    def get_nearest_tasks(self, agent):
        nearest_task = None
        # Finds the furthest possible distance between two points of a rectangluar grid.
        nearest_dist = math.sqrt(math.pow(self.size[0], 2) + math.pow(self.size[1], 2))
        for task in agent.task_queue.get_queue():
            dist = math.sqrt(math.pow(task.pose.position.x - agent.pose.position.x, 2)
                             + math.pow(task.pose.position.y - agent.pose.position.y, 2)
                             + math.pow(task.pose.position.z - agent.pose.position.z, 2))
            if dist < nearest_dist:
                nearest_task = task
        # Ensures no "current task" is assigned to multiple agents,
        self.tasks.remove(nearest_task)
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
        self.add_agent(ConstructionAgent('Construction Agent 1', self.pose_constructor(0, 0, 0, 0, 0, 0, 1)))
        self.add_agent(ConstructionAgent('Construction Agent 2', self.pose_constructor(0, 1, 0, 0, 0, 0, 1)))
        self.add_agent(ConstructionAgent('Construction Agent 3', self.pose_constructor(0, 2, 0, 0, 0, 0, 1)))

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
                                       pose=self.pose_constructor(10, 4, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 2', "Construction required in this area",
                                       pose=self.pose_constructor(15, 10, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 3', "Construction required in this area",
                                       pose=self.pose_constructor(20, 27, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 4', "Construction required in this area",
                                       pose=self.pose_constructor(20, 14, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 5', "Construction required in this area",
                                       pose=self.pose_constructor(75, 60, 0, 0, 0, 0, 1)))
        self.add_task(ConstructionTask('Construction Task 6', "Construction required in this area",
                                       pose=self.pose_constructor(4, 12, 0, 0, 0, 0, 1)))





