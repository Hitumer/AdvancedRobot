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
from scipy.linalg import null_space

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
    # pos = list(np.random.uniform(position_min, position_max))
    pos = [-0.13, 0.25, 0.25]
    if not agent.arm.check_arm_collision(obstacle):
        obstacle.set_position(pos)
    return obstacle, pos

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
    input('Press enter to finish ...')
    # cuboid = Shape.create(type=PrimitiveShape.CUBOID,
    #                   size=[0.6, 0.6, 0.6])
    # pos = [0.7, 0.0, 0.3]
    # cuboid.set_position(pos)
    # cuboid.set_dynamic(False)
    # cuboid.set_collidable(True)
    # cuboid.set_respondable(False)
    # pr.step()
    
    # cylinder = Shape.create(type=PrimitiveShape.CYLINDER,
    #                 size=[0.01, 0.01, 0.06])
    # pos = [0.41, 0.0, 0.63]
    # cylinder.set_position(pos)
    # cylinder.set_dynamic(False)
    # cylinder.set_collidable(True)
    # cylinder.set_respondable(True)
    # cylinder.set_detectable(False)
    # pr.step()
    
    joint_pos = arm.get_joint_positions()
        
    pos_path = path_to(agent, [0.41, 0.0, 0.63], ori = [math.radians(90), math.radians(90), math.radians(90)])
    start_pos = agent.arm.get_tip().get_position() 
    start_ori = agent.arm.get_tip().get_orientation() 
    joint_7th_pos_start = read_joint_cart_pos(agent)[6]

    # for i in range(50):
    #     agent.gripper.actuate(0.0, 5.0)
    #     pr.step()
    # # else:
    # #     agent.gripper.grasp(cylinder)
    
    
    agent.arm.set_control_loop_enabled(False)
    agent.arm.set_model_dynamic(True)
    pr.step()
    
    obs, obs_pos = generate_obstacle(agent)
    pr.step()
    
    # for num_obs in range(20):
        
    # obs_pos = change_obstacle_position(obs,agent)
    # pr.step()
    
    # negative joint vel, robot goes to the left
    # positive joint vel, robot goes to the right
    # y positive, obstacle on the left
    # y negative, obstacle on the right
    done = False     
    K_d_pos = 0.001 # distance gain
    K_d_ori = 0.1 # distance gain
    K_v = .001 # velocity gain
    j = 0 # iteration index
    while not done:
        
        joint_7th_pos_init = read_joint_cart_pos(agent)[6]           
    
        init_pos = agent.arm.get_tip().get_position() 
        init_ori = agent.arm.get_tip().get_orientation() 
        
        # if init_ori[0] <= -math.pi/2:
        #     init_ori[0] = init_ori[0] + math.pi
        # elif init_ori[0] >= math.pi/2:
        #     init_ori[0] = init_ori[0] - math.pi
        # if init_ori[2] <= -math.pi/2:
        #     init_ori[2] = init_ori[2] + math.pi
        # elif init_ori[2] >= math.pi/2:
        #     init_ori[2] = init_ori[2] - math.pi
        
        jaco = get_jaco(agent)
        jaco_pinv = pinv(jaco[:,0:6])
        null_proj = get_null_proj(jaco[:,0:6])

        # generate nullspace velocity
        joint_cart_pos = read_joint_cart_pos(agent)
        obs_dist = np.zeros(len(joint_cart_pos[0:6]))
        for i in range(len(joint_cart_pos[0:6])):
            obs_dist[i] = math.dist(joint_cart_pos[i].T,np.array(obs_pos))
        obs_dist_max = np.max(obs_dist)
        """"""
        last_obs_dist_max = obs_dist_max
        """"""
        obs_dist_norm = obs_dist / obs_dist_max
        if obs_pos[1] >= 0:
            q_dot_null = K_v * obs_dist_norm
        else:
            q_dot_null = -K_v * obs_dist_norm
            

        
        # distance = np.concatenate((init_pos, init_ori)) - np.concatenate((start_pos, start_ori))
        # x_dot = -K_d * np.array(distance)
        # distance_pos = init_pos - start_pos
        # distance_ori = init_ori - start_ori
        # x_dot_pos = -K_d_pos * np.array(distance_pos) 
        # x_dot_ori = -K_d_ori * np.array(distance_ori)
        # x_dot = np.concatenate((x_dot_pos, x_dot_ori))
        # distance = joint_6th_pos_init - joint_6th_pos_start
        x_dot_joint = -5 * np.array(joint_7th_pos_init - joint_7th_pos_start)
        x_dot_pos = -K_d_pos * np.array(init_pos - start_pos)
        x_dot_ori = -K_d_ori * np.array(init_ori - start_ori)

        
        q_dot_origin = jaco_pinv[:, 0:3] @ x_dot_joint 
        # + pinv(jaco)[:,0:3] @ x_dot_pos + pinv(jaco)[:,3:6] @ x_dot_ori
        # jaco_null = null_space(jaco)
        # q_dot_origin = 100*jaco_null[:,0]+jaco_pinv @ x_dot
        # result = agent.arm.set_joint_target_velocities(q_dot_origin)
        # q = arm.get_joint_target_positions()
        # result = arm.set_joint_target_positions(q+q_dot_origin*0.01*10000)
        
        # q_dot_origin = q_dot_origin - q_dot_origin
        
        q_dot_desired = q_dot_origin + null_proj @ q_dot_null[0:6] #  velocity in joint space
        q_dot_desired = np.concatenate((q_dot_desired, [0]))
        q_dot_desired = q_dot_desired + pinv(jaco)[:,0:3] @ x_dot_pos
        #  + pinv(jaco)[:,3:6] @ x_dot_ori
        result = agent.arm.set_joint_target_velocities(q_dot_desired)
        
        

        pr.step()
        j += 1

        joint_cart_pos = read_joint_cart_pos(agent)
        obs_dist = np.zeros(len(joint_cart_pos))
        for i in range(len(joint_cart_pos)):
            obs_dist[i] = math.dist(joint_cart_pos[i].T,np.array(obs_pos))
        obs_dist_max = np.max(obs_dist)

        if j == 500:
            done = True
        # elif obs_dist_max - last_obs_dist_max  <=0 :
        #     done = True
        


        # print("size of null space projector is:", size_of_null_proj)

    input('Press enter to finish ...')
    pr.stop()  # Stop the simulation
    pr.shutdown()  # Close the application
