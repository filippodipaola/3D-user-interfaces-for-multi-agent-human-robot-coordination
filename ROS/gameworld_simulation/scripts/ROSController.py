#!/usr/bin/env python
from gameworld_simulation.msg import Agent as AgentMsg
from gameworld_simulation.msg import ConstructionTask as ConTask
from gameworld_simulation.msg import TransportTask as TransTask
from gameworld_simulation.msg import Vector3
from Timer import Timer
import time
from geometry_msgs.msg import Pose
import Agents
import rospy
import Tasks
from std_msgs.msg import String

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ROSController():
    __metaclass__ = Singleton

    def __init__(self):
        rospy.init_node('gameworld_controller',anonymous=True)
        self.agent_publisher = rospy.Publisher('gameworld/agents', AgentMsg, queue_size=10)
        self.transport_task_publisher = rospy.Publisher('gameworld/transport_tasks', TransTask, queue_size=25)
        self.construct_task_publisher = rospy.Publisher('gameworld/construct_tasks', ConTask, queue_size=25)
        self.updatemsg_publisher = rospy.Publisher('gameworld/update_messages', String, queue_size=10)
        self.unity_subscriber = rospy.Subscriber('Unity3D/update_message', String, self.update_message_subscriber)
        self.user_input_subscriber = rospy.Subscriber('Unity3D/user_input', String, self.user_input_subscriber)
        self.game_start = False
        self.got_end_confirmation = False
        self.recieved_move_message = False
        self.recieved_assign_message = False
        self.user_assign_task = ["agent_id", "task_id"]
        self.user_assign_move = ["agent_id", "pose"]
        print("ROS Controller has started, waiting for message to be recieved from UNITY")


    def publish_string_update_message(self, string):
        my_message = String()
        complete_string = "display message|" + string
        my_message.data = complete_string
        self.publish_update_message(my_message)

    def publish_update_message(self, message):
        self.updatemsg_publisher.publish(message)

    def update_message_subscriber(self, message):
        string_message = message.data
        print("Message RECIEVED from subscriber: %s" % string_message)
        if string_message == "start":
            print("Start message recieved from UNITY, starting simulation.")
            self.game_start = True
            msg = String()
            msg.data = "start command recieved"
            self.publish_update_message(msg)
        if string_message == "pause":
            self.game_start = False

        if string_message == "finished confirmed":
            print("UNITY HAS CONFIRMED THAT IT IS FINISHED!")
            self.got_end_confirmation = True;

    def publish_tasks(self, tasks):
        for t in tasks:
            if isinstance(t, Tasks.ConstructionTask):
                self.construct_task_publisher.publish(self.task_message_constructor(t))
            elif isinstance(t, Tasks.TransportTask):
                self.transport_task_publisher.publish(self.task_message_constructor(t))
            time.sleep(0.005)
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
                #task_message.agent_holding = TransportAgent("None", pose=fakePose)
        return task_message



    def publish_agents(self, agents):
        for a in agents:
            self.agent_publisher.publish(self.agent_message_constructor(a))
            time.sleep(0.005)

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
        if not agent.task_queue.is_empty():
            agent_message.current_task = agent.get_task().name
        else:
            agent_message.current_task = "None"
        return agent_message

    def user_input_subscriber(self, message):
        timer = Timer()
        print ("GOT USER INPUT: %s" % message)
        user_input = message.data
        split_input = user_input.split('|')
        if split_input[0] == "MOVE":
            self.recieved_move_message = True
            print [split_input[1], split_input[2].split(",")]
            self.user_assign_move = [split_input[1], split_input[2].split(",")]
        if split_input[0] == "ASSIGN":
            self.recieved_assign_message = True
            self.user_assign_task = [split_input[1], split_input[2]]
            self.publish_string_update_message("%s has been assigned to %s." % (split_input[1], split_input[2]))


