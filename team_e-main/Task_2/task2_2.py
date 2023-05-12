from cmath import pi
import math
import numpy as np
import roboticstoolbox as rtb
from pyrep import PyRep
from pyrep.backend import sim
from pyrep.const import JointMode
from pyrep.const import ObjectType
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.const import PrimitiveShape
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from task2_help_function import *

class MyRobot(object):
  def __init__(self, arm, gripper):
    self.arm = arm
    self.gripper = gripper

position_min, position_max = [-0.5, -0.5, 0.25], [0.5, 0.5, 0.75]

def generate_obstacle(agent):
    obstacle = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.02, 0.02, 0.02],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)
    pos = list(np.random.uniform(position_min, position_max))
    if not agent.arm.check_arm_collision(obstacle):
        obstacle.set_position(pos)
    return obstacle

def change_obstacle_position(obstacle,agent):
    pos = list(np.random.uniform(position_min, position_max))
    if not agent.arm.check_arm_collision(obstacle):
        obstacle.set_position(pos)
    return pos

if __name__ == '__main__':
  
    pr = PyRep()
    # Launch the application with a scene file that contains a robot
    # pr.launch(f'./scenes/panda_empty.ttt', headless=False)
    pr.launch(f'./scenes/panda_cylinder.ttt', headless=False)
    pr.start()  # Start the simulation
    # initialize the agent
    arm = Panda()  # Get the panda from the scene
    gripper = PandaGripper()  # Get the panda gripper from the scene
    agent = MyRobot(arm,gripper)
    # rtb model to calculate the jocobian of robot
    robot = rtb.models.DH.Panda()
    
    agent.arm.set_control_loop_enabled(True)

    cuboid = Shape.create(type=PrimitiveShape.CUBOID,
                      size=[0.6, 0.6, 0.6])
    pos = [0.7, 0.0, 0.3]
    cuboid.set_position(pos)
    cuboid.set_dynamic(False)
    cuboid.set_collidable(True)
    cuboid.set_respondable(False)
    pr.step()
    
    cylinder = Shape.create(type=PrimitiveShape.CYLINDER,
                    size=[0.01, 0.01, 0.06])
    pos = [0.41, 0.0, 0.63]
    cylinder.set_position(pos)
    cylinder.set_dynamic(False)
    cylinder.set_collidable(True)
    cylinder.set_respondable(True)
    cylinder.set_detectable(False)
    pr.step()
    # cuboid.set_orientation([math.radians(90), math.radians(90), math.radians(90)])
    
    # start_pos = agent.arm.get_tip().get_position() 
    pos_path = path_to(agent, [0.41, 0.0, 0.63], ori = [math.radians(90), math.radians(90), math.radians(90)])
    start_pos = agent.arm.get_tip().get_position() 
    start_ori = agent.arm.get_tip().get_orientation() 

    for i in range(50):
        agent.gripper.actuate(0.0, 5.0)
        pr.step()
    # else:
    #     agent.gripper.grasp(cylinder)
    
    
    # agent.arm.set_control_loop_enabled(False)
    agent.arm.set_model_dynamic(True)
    pr.step()
    
    obs = generate_obstacle(agent)
    pr.step()
    
    for num_obs in range(20):
        
        obs_pos = change_obstacle_position(obs,agent)
        pr.step()
        
        # negative joint vel, robot goes to the left
        # positive joint vel, robot goes to the right
        # y positive, obstacle on the left
        # y negative, obstacle on the right
        done = False
        K_d_pos = 5 # distance gain
        K_d_ori = 0.2 # distance gain
        K_v = 0.2# velocity gain
        j = 0 # iteration index
        while not done:
            init_pos = agent.arm.get_tip().get_position() 
            init_ori = agent.arm.get_tip().get_orientation() 
            jaco = get_jaco(agent)
            jaco_pinv = pinv(jaco)
            null_proj = get_null_proj(jaco)

            # generate nullspace velocity
            joint_cart_pos = read_joint_cart_pos(agent)
            obs_dist = np.zeros(len(joint_cart_pos))
            for i in range(len(joint_cart_pos)):
                obs_dist[i] = math.dist(joint_cart_pos[i].T,np.array(obs_pos))
            obs_dist_max = np.max(obs_dist)
            """"""
            last_obs_dist_max = obs_dist_max
            """"""
            obs_dist_norm = obs_dist / obs_dist_max
            if obs_pos[1] >= 0:
                q_dot_null = K_v * obs_dist_norm
            else:
                q_dot_null = -K_v* obs_dist_norm

            # distance = init_pos - start_pos
            # x_dot = -K_d * np.array(distance)
            # q_dot_origin = jaco_pinv[:, 0:3] @ x_dot
            
            # distance = np.concatenate((init_pos, init_ori)) - np.concatenate((start_pos, start_ori))
            # x_dot = -K_d * np.array(distance)
            distance_pos = init_pos - start_pos
            distance_ori = init_ori - start_ori
            x_dot_pos = -K_d_pos * np.array(distance_pos)
            x_dot_ori = -K_d_ori * np.array(distance_ori)
            x_dot = np.concatenate((x_dot_pos, x_dot_ori))
            
            q_dot_origin = jaco_pinv @ x_dot
            
            q_dot_desired = q_dot_origin + null_proj @ q_dot_null #  velocity in joint space
            result = agent.arm.set_joint_target_velocities(q_dot_desired)

            pr.step()
            j += 1

            joint_cart_pos = read_joint_cart_pos(agent)
            obs_dist = np.zeros(len(joint_cart_pos))
            for i in range(len(joint_cart_pos)):
                obs_dist[i] = math.dist(joint_cart_pos[i].T,np.array(obs_pos))
            obs_dist_max = np.max(obs_dist)

            if j == 100:
                done = True
            elif obs_dist_max - last_obs_dist_max  <=0 :
                done = True


        # print("size of null space projector is:", size_of_null_proj)

    input('Press enter to finish ...')
    pr.stop()  # Stop the simulation
    pr.shutdown()  # Close the application
