#!/usr/bin/env python
from gameworld_simulation.msg import Agent as AgentMsg
from gameworld_simulation.msg import ConstructionTask as ConTask
from gameworld_simulation.msg import TransportTask as TransTask
from gameworld_simulation.msg import Vector3
import Agents
import rospy
import Tasks
from std_msgs.msg import String

class ROSController():

    def __init__(self):
        rospy.init_node('gameworld_controller',anonymous=True)
        self.agent_publisher = rospy.Publisher('gameworld/agents', AgentMsg, queue_size=10)
        self.transport_task_publisher = rospy.Publisher('gameworld/transport_tasks', TransTask, queue_size=25)
        self.construct_task_publisher = rospy.Publisher('gameworld/construct_tasks', ConTask, queue_size=25)
        self.updatemsg_publisher = rospy.Publisher('gameworld/update_messages', String, queue_size=10)



    def publish_update_message(self, message):
        self.updatemsg_publisher.publish(message)


    def publish_tasks(self, tasks):
        for t in tasks:
            if isinstance(t, Tasks.ConstructionTask):
                self.construct_task_publisher.publish(self.task_message_constructor(t))
            elif isinstance(task, Tasks.TransportTask):
                self.transport_task_publisher.publish(self.task_message_constructor(t))
    #TODO Need to test if this can work with just a task message, instead of two different messages.
    def task_message_constructor(self, task):
        if isinstance(task, Tasks.ConstructionTask):
            task_message = ConTask()
        elif isinstance(task, Tasks.TransportTask):
            task_message = TransTask()
        task_message.name = task.name
        task_message.description = task.description
        task_message.capabilities_needed = [x.value for x in task.capabilties_needed]
        task_message.pose = task.pose
        task_message.is_complete = task.is_complete
        if isinstance(task, Tasks.ConstructionTask):
            task_message.task_type = "ConstructionTask"
            task_message.work_needed = task.work_needed
            task_message.work_done = task.work_done
        elif isinstance(task, Tasks.TransportTask):
            task_message.task_type = "TransportTask"
            task_message.start_pose = task.start_pose
            task_message.end_pose = task.end_pose
            task_message.agent_holding = task.agent_holding
        return task_message



    def publish_agents(self, agents):
        for a in agents:
            self.agent_publisher.publish(self.agent_message_constructor(a))

    def agent_message_constructor(self, agent):
        agent_message = AgentMsg()
        agent_message.name = agent.name
        agent_message.current_action = agent.current_action
        agent_message.movement_speed = agent.movement_speed
        agent_message.pose = agent.pose
        size = Vector3()
        size.x = agent.size[0]
        size.y = agent.size[1]
        size.z = agent.size[2]
        agent_message.size = size
        agent_message.capabilities = [x.value for x in agent.capabilities]
        if isinstance(agent,Agents.ConstructionAgent):
            agent_message.agent_type = "ConstructionAgent"
            agent_message.construction_speed = agent.construction_speed
        elif isinstance(agent, Agents.TransportAgent):
            agent_message.agent_type = "TransportAgent"
            agent_message.construction_speed = -1.0

        agent_message.is_at_task = agent.is_at_task
        agent_message.current_task = agent.get_task().__str__()
        return agent_message



