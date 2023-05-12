import roboticstoolbox as rtb
import numpy as np
import math
from pyrep.const import ObjectType
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep import PyRep
from pyrep.errors import ConfigurationPathError
from pyrep.errors import IKError
from spatialmath.base import *

pr = PyRep()
robot = rtb.models.DH.Panda()


def path_to(agent,pos,ori):
    try:
        path = agent.arm.get_path(
            position=pos, euler=ori, \
                algorithm=Algos.EST)
        done = False
        path.visualize()
        while not done:
            done = path.step()
            pr.step() 
        for i in range(50):
           pr.step()
    except ConfigurationPathError as e:
        print('Could not find path')
    return path
    

def linear_path_to(agent,pos,ori):
    try:
        path = agent.arm.get_linear_path(
            position=pos, euler=ori, \
                )
        done = False
        path.visualize()
        while not done:
            done = path.step()
            pr.step() 
        for i in range(50):
           pr.step()
    except ConfigurationPathError as e:
        print('Could not find path')
    return path

def get_jaco_r(agent):
    # jacobian of agent_right
    init_pos = agent.arm.get_tip().get_position() 
    (x, y, z), q = init_pos, agent.arm.get_tip().get_quaternion()
    try:
        init_joint_pos = agent.arm.solve_ik_via_jacobian([x, y, z], quaternion=q)
    except IKError:
        init_joint_pos = agent.arm.solve_ik_via_sampling([x, y, z], quaternion=q)[0]
    # calculate the null-space projector
    jaco = robot.jacob0(init_joint_pos) # correct in dimension
    T_r=transl(0,0.2,0.6)@rpy2tr(0,math.radians(90),math.radians(-90),order='xyz')#,@,trotx(-90,,'deg')
    R_r=T_r[0:3,0:3]
    trans_1=np.hstack((R_r,np.zeros((3,3))))
    trans_2=np.hstack((np.zeros((3,3)),R_r))
    trans=np.vstack((trans_1,trans_2))
    jaco = trans@jaco
    return jaco

def get_jaco_l(agent):
    # jacobian of agent_right
    init_pos = agent.arm.get_tip().get_position() 
    (x, y, z), q = init_pos, agent.arm.get_tip().get_quaternion()
    try:
        init_joint_pos = agent.arm.solve_ik_via_jacobian([x, y, z], quaternion=q)
    except IKError:
        init_joint_pos = agent.arm.solve_ik_via_sampling([x, y, z], quaternion=q)[0]
    # calculate the null-space projector
    jaco = robot.jacob0(init_joint_pos) # correct in dimension
    T_r=transl(0,0.2,0.6)@rpy2tr(0,math.radians(90),math.radians(90),order='xyz')#,@,trotx(-90,,'deg')
    R_r=T_r[0:3,0:3]
    trans_1=np.hstack((R_r,np.zeros((3,3))))
    trans_2=np.hstack((np.zeros((3,3)),R_r))
    trans=np.vstack((trans_1,trans_2))
    jaco = trans@jaco
    return jaco

def pinv(jaco):
    jaco_pseudo_inv = np.linalg.pinv(jaco, rcond=1e-3)
    return jaco_pseudo_inv





