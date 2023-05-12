import math
import numpy as np
from pyrep import PyRep
import roboticstoolbox as rtb
from pyrep.backend import sim, utils
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
# self defined function
from task3_help_function import path_to
from task3_help_function import linear_path_to

class MyRobot(object):
  def __init__(self, arm, gripper):
    self.arm = arm
    self.gripper = gripper


def main():
    pr = PyRep()

    # Launch application
    pr.launch('./scenes/frankaSim_bimanual_test.ttt', headless=False) 
    
    # Start simulation
    pr.start()
       
    # Get the panda from the scene
    arm_left = Panda(0)
    arm_right = Panda(1)
    gripper_left = PandaGripper(0)
    gripper_right = PandaGripper(1)  # Get the panda gripper from the scene
    agent_left = MyRobot(arm_left,gripper_left)
    agent_right = MyRobot(arm_right,gripper_right)
    # robotics tool box
    robot = rtb.models.DH.Panda()

    # save the initial position of eef
    init_pos_right = agent_right.arm.get_tip().get_position()
    init_quat_right = agent_right.arm.get_tip().get_quaternion()
    init_pos_left = agent_left.arm.get_tip().get_position()
    init_quat_left = agent_left.arm.get_tip().get_quaternion()

    # get waypoint from scene
    ctrlPt, ctrlPt0, ctrlPt1, ctrlPt2  \
        = Dummy("ctrlPt"), Dummy("ctrlPt0"), Dummy("ctrlPt1"), Dummy("ctrlPt2")
    waypoint = np.array([ctrlPt.get_position(), \
        ctrlPt0.get_position(),ctrlPt1.get_position(), ctrlPt2.get_position()])
    # get object from scene
    cuboid = Shape("Cuboid") # from pyrep.objects.shape import Shape
    desired_pos = cuboid.get_position()
    
    '''
    right arm moves
    '''
    ## right arm pose path
    pose_path_right = path_to(agent_right,pos = init_pos_right, ori = [0, math.radians(180), math.radians(90)] )

    ## right arm move to the cuboid
    grasp_path_right = linear_path_to(agent_right, desired_pos, ori = [0, math.radians(180), math.radians(90)])

    # right arm grasps
    cuboid.set_dynamic(True)
    cuboid.set_collidable(True)
    cuboid.set_respondable(True)
    for i in range(50):
        agent_right.gripper.actuate(0.0, 5.0)
        pr.step()
    else:
        agent_right.gripper.grasp(cuboid)

    ## left arm goes near to waypoint and prepare to grasp
    (x,y,z) = waypoint[0]
    desired_pos_left = (x,y-0.2,z)
    pos_path_left = path_to(agent_left, desired_pos_left, ori = [math.radians(90), math.radians(180), math.radians(-90)])

    # right arm goes to waypoint 1 (index 0) position
    cuboid.set_detectable(False)
    cuboid.set_collidable(False)
    return_path_right = path_to(agent_right, waypoint[0], ori = [math.radians(90), math.radians(0), math.radians(180)])
    
    ## left arm grasp path
    cuboid.set_dynamic(True)
    cuboid.set_collidable(True)
    cuboid.set_respondable(True)
    desired_pos_left = cuboid.get_position()
    grasp_path_left = linear_path_to(agent_left, desired_pos_left, ori = [math.radians(90), math.radians(180), math.radians(-90)])
    # left arm grasps
    for i in range(50):
        agent_left.gripper.actuate(0.0, 5.0)
        pr.step()
    else:
        agent_left.gripper.grasp(cuboid)
    # check whether both arms grasp the cuboid
    if agent_left.gripper.grasp(cuboid) and agent_right.gripper.grasp(cuboid):
        print("cuboid grasped")
       
       
    '''
    detach the cuboid
    '''
    # right arm detaches
    for i in range(50):
        agent_right.gripper.actuate(1.0, 5.0)
        pr.step()
    else:
        agent_right.gripper.release()
    # left arm moves
    cuboid.set_detectable(False)
    cuboid.set_collidable(False)
    desired_pos_left_new = (x,y-0.2,z)
    away_path_left = path_to(agent_left, desired_pos_left_new, ori = [math.radians(90), math.radians(180), math.radians(-90)])
    
    # left arm detaches
    for i in range(50):
        agent_left.gripper.actuate(1.0, 5.0)
        pr.step()
    else:
        agent_left.gripper.release()
    


    for i in range(50):
        pr.step()
    
    input('Press enter to finish ...')

    # Stop the simulation
    pr.stop()  

    # Close the application
    pr.shutdown() 


if __name__=="__main__":
    main()