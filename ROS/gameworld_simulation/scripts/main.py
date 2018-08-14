#!/usr/bin/env python

from GameWorld import GameWorld
import time

def main():
    #rospy.init_node('gameworld_controller', anonymous=True)
    game_world = GameWorld()
    while(True):
        game_world.update()
        # Needs to be changed to ros spin with a update time Hz
        time.sleep(0.1)



if __name__ == "__main__":
    main()
else:
    print("Shit's not right!")