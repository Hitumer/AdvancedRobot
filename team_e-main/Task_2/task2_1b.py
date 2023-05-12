import math
import numpy as np
from pyrep import PyRep
from pyrep.backend import sim, utils
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape
from pyrep.robots.arms.panda import Panda
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError

class PandaWithGrippper(Panda):
    def __init__(self, *kwargs):
        super().__init__(*kwargs)
        self.gripper_joints = [Joint("Panda_gripper_joint1"), Joint("Panda_gripper_joint2")]

    def close_gripper(self, open=False):
        sign = -1.0 if not open else 1.0
        for j in self.gripper_joints:
            j.set_joint_target_velocity(sign * 0.1)


if __name__ == '__main__':


    pr = PyRep()
    pr.launch(f'/home/btknzn/Desktop/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/PyRep/examples/team_e-Assignment-2.1/scenes/panda_reach_cylinder_cluttered.ttt', headless=False) 
    pr.start()

    agent = PandaWithGrippper()

    q_start = np.array([0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4])
    agent.set_joint_positions(q_start)

    cylinder = Shape("Cylinder")
    target_pos = cylinder.get_position()
    
    # Get a path to the target (rotate so z points down)

    path = agent.get_nonlinear_path(
                position=target_pos, euler=[0, math.radians(180), 0], max_configs=5, max_time_ms=3000, trials_per_goal=5)


    # Step the simulation and advance the agent along the path
    done = False
    path.visualize()
    while not done:
        done = path.step()
        pr.step()

    agent.close_gripper()
    for i in range(50):
        pr.step()

    try:
        cylinder.set_detectable(False)
        cylinder.set_collidable(False)

        target_pos[0] -= 0.15
        target_pos[2] += 0.05
        path = agent.get_linear_path(position=target_pos, euler=[0, math.radians(180), 0])

        cylinder.set_detectable(True)
        cylinder.set_collidable(True)
        cylinder.set_respondable(True)
        cylinder.set_measurable(True)
        done = False
        path.visualize()
        while not done:
            done = path.step()
            pr.step()

        for i in range(50):
            pr.step()
            
    except ConfigurationPathError as e:
        print('Could not find path')

    pr.stop()  # Stop the simulation
    pr.shutdown()  # Close the application
