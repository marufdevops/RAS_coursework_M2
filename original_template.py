#
# The TiaGo robot is expected to approach four of the items in the environment.
# It must do so, avoid collisions with any of the static or mobile obstacles
# (i.e., walls, other robots and the cones), and it must reach the goals
# regardless of its starting position. The possible goal items are:
#  - the wooden box with the red top: `red`
#  - the wooden box with the green top: `green`
#  - the container with the ducks: `ducks`
#  - the container with the balls: `balls`
#
# The objects will be considered "close" when the distance to the TiaGo robot
# is lower than 0.8m.
#
# The input will be given using the keyboard, typing the previously mentioned
# shortened names (e.g., red, green, ducks, balls) in the 3D view (remember
# to click inside it) and pressing enter. Goals can be provided separated by
# commas. If new goals are provided before meeting the current goals, the new
# ones must be queued (the existing ones must be satisfied before continuing).
#
# The documentation is expected to be in the code, not an external document.
#

import math
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range

robot = Robot()

timestep = int(robot.getBasicTimeStep())


l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

keyboard = KeyboardReader(timestep)

while (robot.step(timestep) != -1):
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')

    # print('The robot is next to', get_goals_in_range(*gps.getValues()[0:2]))
    


