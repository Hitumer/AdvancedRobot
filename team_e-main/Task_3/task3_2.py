import errno
import math
from sys import path
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
from task3_help_function import *

class MyRobot(object):
  def __init__(self, arm, gripper):
    self.arm = arm
    self.gripper = gripper


def main():
    pr = PyRep()

    # Launch application
    """
    the path is modified
    """
    pr.launch('./scenes/frankaSim_bimanual_test_.ttt', headless=False) 
    
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
    
    ### right arm
    ## poes
    pose_path_right = path_to(agent_right,pos = init_pos_right, ori = [0, math.radians(180), math.radians(90)] ) 

    ## grasp
    desired_pos_right = cuboid.get_position()
    grasp_path_right = linear_path_to(agent_right, desired_pos_right, ori = [0, math.radians(180), math.radians(90)])
    cuboid.set_dynamic(True)
    cuboid.set_collidable(True)
    cuboid.set_respondable(True)


    # right hand grasps
    for i in range(50):
        agent_right.gripper.actuate(0.0, 5.0)
        pr.step()
    else:
        agent_right.gripper.grasp(cuboid)
    
    ### left arm
    (x,y,z) = waypoint[0]
    desired_pos_left = (x,y-0.2,z)
    ## pose
    pos_path_left = path_to(agent_left, desired_pos_left, ori = [math.radians(90), math.radians(180), math.radians(-90)])

    ## right arm return
    cuboid.set_detectable(False)
    cuboid.set_collidable(False)
    return_path_right = path_to(agent_right, waypoint[0], ori = [math.radians(90), math.radians(0), math.radians(180)])
    
    ## left arm grasp
    grasp_path_left = linear_path_to(agent_left, waypoint[0], ori = [math.radians(90), math.radians(180), math.radians(-90)])
    # left hand grasps
    for i in range(50):
        agent_left.gripper.actuate(0.0, 5.0)
        pr.step()
    else:
        agent_left.gripper.grasp(cuboid)
    """
    while not agent_left.gripper.grasp(cuboid):
        agent_left.gripper.actuate(0.0, 10.0)
        pr.step()
    """
    ## make sure both arms grasp the cuboid

    if agent_left.gripper.grasp(cuboid) and agent_right.gripper.grasp(cuboid):
        print("cuboid grasped")
        pr.step()
    
    pr.step()

    cuboid.set_dynamic(True)
    cuboid.set_collidable(True)
    cuboid.set_respondable(True)
    cuboid.set_bullet_friction(1000)
    agent_right.gripper.set_bullet_friction(1000)
    agent_left.gripper.set_bullet_friction(1000)
    """"""
    """
    task 3.2 
    """
    # compensate the agent_left.arm.get_tip().get_orientation error
    rot_left = agent_left.arm.get_tip().get_orientation()
    rot_left_delta = [math.radians(90), math.radians(180), math.radians(-90)] - rot_left
    x_left = x_right= np.zeros(6)

    J_a = 0.5 * np.hstack((np.eye(6),np.eye(6)))
    J_a_pinv = J_a.T @ np.linalg.inv(J_a @ J_a.T)   # pseudo-inverse
    J_r = np.hstack((-1 * np.eye(6),np.eye(6)))
    J_r_pinv = J_r.T @ np.linalg.inv(J_r @ J_r.T)
    
    
    """sample waypoint from a circle"""
    center_circle = np.mean(waypoint, axis = 0)
    radius_circle = waypoint[1][1] - waypoint[0][1]
    num_sample = 16
    waypoint_new = np.empty(shape = (num_sample,3))
    for i in range(num_sample):
        waypoint_new[i][0] = waypoint[0][0]
        waypoint_new[i][1] = radius_circle * math.sin(i*2*math.pi/num_sample) + center_circle[1]
        waypoint_new[i][2] = radius_circle * math.cos(i*2*math.pi/num_sample) + center_circle[2]
    # waypoint_new[0][:] = waypoint[0][:]
    # waypoint_new[num_sample//4][:] = waypoint[1][:]
    # waypoint_new[num_sample//2][:] = waypoint[2][:]
    # waypoint_new[num_sample*3//4][:] = waypoint[3][:]
    
        

    for i in range(32): # (len(waypoint))      
        waypoint_index = (i+1)%num_sample
        done = False
        print("arms move to waypoint", waypoint_index+1)
        while not done:           
            agent_left.gripper.grasp(cuboid)
            agent_right.gripper.grasp(cuboid)

            target_pos = waypoint_new[waypoint_index] 
            """
            first layer
            """
            target_rot_right = [math.radians(90), math.radians(0), math.radians(180)]
            target_rot_left = [math.radians(90), math.radians(180), math.radians(-90)]
            # right = 1. left = 2
            pos_left = agent_left.arm.get_tip().get_position()
            pos_right = agent_right.arm.get_tip().get_position()
            rot_left = agent_left.arm.get_tip().get_orientation()
            rot_left = rot_left + rot_left_delta
            rot_right = agent_right.arm.get_tip().get_orientation()
            

            x = np.hstack((pos_right, rot_right, pos_left, rot_left))
            x_a = J_a @ x
            x_r = J_r @ x

            jaco_right = get_jaco_r(agent_right)
            jaco_left = get_jaco_l(agent_left)
            jaco_pinv_right = pinv(jaco_right)      # Pseudo-inverse
            jaco_pinv_left = pinv(jaco_left)

            pos_abs = 0.5 * (pos_left + pos_right)
            pos_dot_abs = -1 * (pos_abs - target_pos)
            # enable velocity control
            agent_right.arm.set_control_loop_enabled(False)
            agent_right.arm.set_model_dynamic(True)
            agent_left.arm.set_control_loop_enabled(False)
            agent_left.arm.set_model_dynamic(True)
            
            x_d = (np.hstack((target_pos,target_rot_right, target_pos, target_rot_left))).T
            x_a_d = J_a @ x_d
            x_r_d = J_r @ x_d

            k_a = 0.44 # 0.44 # 0.5
            k_r = 1.45 # 1.5
            x_dot_a_d = (np.hstack((pos_dot_abs, np.zeros(3)))).T
            x_dot_r_d = np.zeros(6)
            #error_a = x_a_d - J_a @ x_d
            #error_r = x_r_d - J_r @ x_d
            error_a = x_a_d - J_a @ x
            error_r = x_r_d - J_r @ x

            x_dot_d = J_a_pinv @ (x_dot_a_d + k_a * error_a) \
                + J_r_pinv @ (x_dot_r_d + k_r * error_r) 
            
            x_dot_d_right = x_dot_d[0:6] # =
            x_dot_d_left = x_dot_d[6:] # =
            """
            second layer
            """
            error_pos_right = target_pos - pos_right # x
            error_pos_left = target_pos - pos_left # x
            error_rot_right = target_rot_right - rot_right 
            error_rot_left = target_rot_left - rot_left
            error_right = (np.hstack((error_pos_right,error_rot_right))).T
            error_left = (np.hstack((error_pos_left,error_rot_left))).T
            
            q_dot_d_right = jaco_pinv_right @ (x_dot_d_right + 0.1 * error_right)
            q_dot_d_left = jaco_pinv_left @ (x_dot_d_left + 0.1 * error_left)

            agent_right.arm.set_control_loop_enabled(False)
            agent_right.arm.set_model_dynamic(True)
            agent_left.arm.set_control_loop_enabled(False)
            agent_left.arm.set_model_dynamic(True)
            
            agent_right.arm.set_joint_target_velocities(q_dot_d_right)
            agent_left.arm.set_joint_target_velocities(q_dot_d_left)
            
            current_pos = cuboid.get_position()

            pr.step()

            check_goal = abs(current_pos - target_pos) <= 0.01
            if check_goal.all():
                print("arrived waypoint", waypoint_index+1)
                done = True
                q_dot_zero = np.zeros(7)
                agent_right.arm.set_joint_target_velocities(q_dot_zero)
                agent_left.arm.set_joint_target_velocities(q_dot_zero)

                try:
                    joint_pos_right = agent_right.arm.solve_ik_via_jacobian(target_pos, target_rot_right)
                except IKError:
                    joint_pos_right = agent_right.arm.solve_ik_via_sampling(target_pos, target_rot_right)[0]
                try:
                    joint_pos_left = agent_left.arm.solve_ik_via_jacobian(target_pos, target_rot_left)
                except IKError:
                    joint_pos_left = agent_left.arm.solve_ik_via_sampling(target_pos, target_rot_left)[0]

                print("reset arm")
                for reset_step in range(5):
                    agent_right.arm.set_joint_target_positions(joint_pos_right)
                    agent_left.arm.set_joint_target_positions(joint_pos_left)
                    pr.step()
                


    input('Press enter to finish ...')

    # Stop the simulation
    pr.stop()  

    # Close the application
    pr.shutdown() 


if __name__=="__main__":
    main()





"""
desired_pos_left = cuboid.get_position()
    try:
        grasp_path_left = agent_left.arm.get_linear_path(
            position= desired_pos_left, euler=[math.radians(90), math.radians(180), math.radians(90)] \
                )
    except ConfigurationPathError as e:
        print('Could not find path')
    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = grasp_path_left.step()
        pr.step() 
"""

'''
    # try to move to the cuboid by solve IK, but the path is chaostic
    (x, y, z), q = desired_pos, agent_right.arm.get_tip().get_quaternion()
    desired_joint_pos = agent_right.arm.solve_ik_via_sampling([x, y, z], quaternion=q)[0]
    done = False
    while not done:
        agent_right.arm.set_joint_target_positions(desired_joint_pos)
        pr.step()
        current_pos = agent_right.arm.get_tip().get_position() 
        a = (current_pos - desired_pos) <= 0.01
        if a.all():
            done = True
'''

    # verify velocity control
"""
    agent_left.arm.set_model_dynamic(True)
    agent_left.arm.set_control_loop_enabled(False)
    agent_right.arm.set_model_dynamic(True)
    agent_right.arm.set_control_loop_enabled(False)
    q_dot_null = 1*np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1])
    for i in range(50):
        agent_left.arm.set_joint_target_velocities(q_dot_null)
        pr.step()
"""