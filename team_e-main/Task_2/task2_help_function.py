import roboticstoolbox as rtb
import numpy as np
from pyrep import PyRep
from pyrep.const import ObjectType
from pyrep.errors import ConfigurationPathError
from pyrep.const import ConfigurationPathAlgorithms as Algos

pr = PyRep()
robot = rtb.models.DH.Panda()

def read_joint_cart_pos(agent):
    # return 7 joints cartesian position
    agent_object_in_tree = agent.arm.get_objects_in_tree()
    arm_joint = []
    for i in agent_object_in_tree:
        #print(i.get_type())
        if i.get_type() == ObjectType.JOINT:
            arm_joint.append(i)
        elif len(arm_joint) == 7:
            break
    arm_joint_pos = []
    for i in range(len(arm_joint)):
        arm_joint_pos.append(arm_joint[i].get_position())
        #print(arm_joint[i].get_position())
    # del arm_joint_pos[0]
    # arm_joint_pos.append(agent.arm.get_tip().get_position())
    #print(arm_joint_pos)
    #print(agent.arm.get_tip().get_position())
    return arm_joint_pos

def pinv(jaco):
    jaco_pseudo_inv = np.linalg.pinv(jaco, rcond=1e-3)
    return jaco_pseudo_inv

def get_jaco(agent):
    init_pos = agent.arm.get_tip().get_position() 
    (x, y, z), q = init_pos, agent.arm.get_tip().get_quaternion()
    init_joint_pos = agent.arm.solve_ik_via_jacobian([x, y, z], quaternion=q)
    # calculate the null-space projector
    jaco = robot.jacob0(init_joint_pos) # correct in dimension
    return jaco

def get_null_proj(jaco):
    jaco_pseudo_inv = pinv(jaco)
    size_of_null_proj = (jaco_pseudo_inv @ jaco).shape # (7,6)
    null_proj = (np.eye(size_of_null_proj[0]) - jaco_pseudo_inv @ jaco) 
    return null_proj 

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