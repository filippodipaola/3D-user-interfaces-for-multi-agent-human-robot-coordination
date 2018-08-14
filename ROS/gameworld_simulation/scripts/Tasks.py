#!/usr/bin/env python

from geometry_msgs.msg import Pose
from CapabilitiesEnums import CapabiltiesEnums as capEnum

class Task():
    def __init__(self, name, description, capabilties_needed=[], pose=Pose()):
        self.name = name
        self.description = description
        self.capabilties_needed = capabilties_needed
        self.pose = pose
        self.is_complete = False

    def __str__(self):
        return "Task name: %s, Description: %s, Capabilities needed: %s, Current Position: %s" % (self.name,
                                                                                                  self.description,
                                                                                                  self.capabilties_needed,
                                                                                                  self.pose.position)

    # Checks if the list of capabilities of the agent is
    # compatible with the task. If it isn't then the agent
    # cannot complete the task.
    def is_agent_compatible(self,agent):
        capabilities = agent.capabilities
        if len(self.capabilties_needed) == 0:
            return True
        return set(self.capabilties_needed) <= set(capabilities)

    def add_needed_abilities(self, cap_list):
        self.capabilties_needed.extend(cap_list)

    def add_needed_ability(self, cap):
        self.capabilties_needed.append(cap)

    def remove_needed_ability(self, cap):
        self.capabilties_needed.remove(cap)

    # This function REQUIRES an override method.
    def is_completed(self):
        pass


class ConstructionTask(Task):
    def __init__(self, name, description, capabilties_needed=[capEnum.construct, capEnum.drive], pose=Pose(), work_needed=100):
        Task.__init__(self, name,description,capabilties_needed,pose)
        self.work_needed = 100
        self.work_done = 0

    def __str__(self):
        return Task.__str__(self) + " Work Needed: %s, Work Done: %s" %(self.work_needed, self.work_needed)

    def is_completed(self):
        self.is_complete = self.work_needed <= self.work_done
        return self.is_complete

    def constuct(self, agent, multiplier=1.0):
        if not self.is_completed():
            self.work_done += multiplier * agent.construction_speed
            return False
        print("The construction task has been completed already, there is no more work to do.")
        self.is_complete = True
        return True


class TransportTask(Task):

    def __init__(self, name, description, capabilties_needed=[capEnum.pickup, capEnum.transport], pose=Pose(),
                 end_pose=Pose()):
        Task.__init__(self, name, description, capabilties_needed, pose)
        self.start_pose = pose
        self.end_pose = end_pose
        self.is_picked_up = False
        self.agent_holding = None

    def __str__(self):
        return Task.__str__() + "End Position: %s, Has Arrived: %s" %(self.end_pose, self.has_arrived())

    def is_completed(self):
        self.is_complete = self.pose.position == self.end_pose.position
        return self.is_complete

    def update_position(self,position):
        self.pose.position = position


    def pick_up(self, agent):
        self.agent_holding = agent
        self.is_picked_up = True

    def dropped(self):
        self.pose.position = self.agent_holding.pose.position
        self.agent_holding = None
        self.is_picked_up = False


    def delivered(self):

        if not self.is_picked_up:
            print("The package has not been picked up by an agent, it cannot be delivered")
            return False

        if not self.is_completed():
            print("The package is not in the assigned delivery position, the package has been dropped.")
            self.dropped()
            return False

        self.agent_holding = None
        self.is_complete = True
        return True







