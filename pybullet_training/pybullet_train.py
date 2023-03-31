import os
import inspect
import time

import pybullet as p
import pybullet_data
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))


DATA = "trot"
JOINT_ID = [1,2,3,5,6,7,9,10,11,13,14,15]

def step(p, robot, action, angles):
   """ Take a step in simulation.

   Args:
         p (_type_): pyBullet client
         robot (_type_): robot URDF asset
         action (_type_): [base pos, base ori in quat, joint pos]
   """
   lookat = action[0:3]
   distance = 1.0
   yaw = 0
   pitch = -15
   p.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)

   p.resetBasePositionAndOrientation(robot, action[0:3], action[3:7])
   if not angles:
      p.setJointMotorControlArray(robot, JOINT_ID, p.POSITION_CONTROL, action[7:])
   else:
      count = 0
      for i in JOINT_ID:
         p.resetJointState(robot, i, action[7+count])
         count += 1
   p.stepSimulation()
   time.sleep(0.001)


def get_data(csv):
   # ACTION SETUP
   if csv:
      gen_coord = pd.read_csv(f"{DATA}.csv")
      joints = gen_coord.iloc[:,7:]
      joints = joints.to_numpy()

      test = np.load("j_pos.npy")
      test2 = np.load("contact.npy")
      # fig, ax = plt.subplots()
      # ax.plot(range(np.shape(test)[0]), test2[:,0])
      # ax2 = ax.twinx()
      # ax2.plot(range(np.shape(test)[0]), test[:,2], color='red')


      # plot the joint positions from our previous thingy
      # plt.plot(range(np.shape(test)[0]), test[:,2])
      # plt.plot(range(np.shape(joints)[0]), joints[:,2])

      # plt.figure()
      # plt.plot(range(np.shape(test)[0]), test[:,0])
    #   plt.plot(range(np.shape(joints)[0]), joints[:,0])
    #   plt.figure()

      # plt.figure()
      # plt.plot(range(np.shape(test)[0]), test[:,1])
      # plt.plot(range(np.shape(joints)[0]), joints[:,1])
      # plt.show()

    #   x = range(np.shape(test)[0])
    #   plt.plot(x, test[:,2])
    #   plt.plot(x, test2[:,2])
    #   plt.plot(x, test2[:,2] + test[:,2])
    #   plt.show()

      base = gen_coord.iloc[:,0:3]
      base = base.to_numpy()

      quat = gen_coord.iloc[:,4:7]
      quat2 = gen_coord.iloc[:,3:4]
      quat = pd.concat([quat,quat2],axis=1)
      quat = quat.to_numpy()

      actions = np.concatenate((base,quat,joints), axis=1)
      t = gen_coord.shape[0]

   else:
      gallop = np.loadtxt(f"{DATA}.txt")

      base = gallop[:,1:4]
      quat1 = gallop[:,5:8]
      quat2 = gallop[:,4]
      quat2 = quat2[:, np.newaxis]
      joints = gallop[:,8:]

      actions = np.concatenate((base,quat1,quat2,joints), axis=1)
      t = gallop.shape[0]

   return (actions,t)

# simulation steps faster than real-time, so we need to put a delay

## SIMULATION SETUP ----------------------------------------------------------------------------------------
physicsClient = p.connect(p.DIRECT)    # p.DIRECT for non-graphical version, gives visuals of simulation too
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
# disables the GUI rendering
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setPhysicsEngineParameter(numSolverIterations=30)
p.setTimeStep(0.001) 
p.setPhysicsEngineParameter(enableConeFriction=0)
p.changeDynamics(0,-1,restitution=1)

shadow_urdf_path = currentdir + '/urdf/a1/a1_shadow.urdf'
plane = p.loadURDF(currentdir + "/urdf/plane/plane.urdf")
a1 = p.loadURDF(shadow_urdf_path, [0,0,0], [0,0,0,1])

p.resetBasePositionAndOrientation(a1, [0,0,0], [0,0,0,1])
p.resetBaseVelocity(a1, [0,0,0], [0,0,0])


# Get joint information
# print(p.getNumJoints(a1))

for i in range(p.getNumJoints(a1)):
   print(p.getJointInfo(a1, i))


actions, t = get_data(csv=True)

z_diff = actions[:, 2] + actions[:,9]
# plt.figure()
# plt.plot(range(np.shape(z_diff)[0]), actions[:,2])
# plt.show()
test = []

contact = []

# foot_z_pos = np.zeros((1,4))
foot_position_robot = np.zeros((1,12))

# RUN SIMULATION
for i in range(t):
   step(p, a1, actions[i,:], False)
   test = np.append(test, (p.getJointStates(a1,[3])[0])[0])

   # check if foot in contact with floor
   try:    
      c_FR = len(None or p.getContactPoints(plane, a1, -1, 4))
   except:
      c_FR = 0
   try:
      c_FL = len(None or p.getContactPoints(plane, a1, -1, 8))
   except:
      c_FL = 0
   try:
      c_RR = len(None or p.getContactPoints(plane, a1, -1, 12))
   except:
      c_RR = 0
   try:
      c_RL = len(None or p.getContactPoints(plane, a1, -1, 16))
   except:
      c_RL = 0

   curr_contact = np.array([c_FR, c_FL, c_RR, c_RL])
   curr_contact = np.where(curr_contact>0,1,0)
   curr_contact = curr_contact[np.newaxis, :]
   if len(contact) is not 0:
      contact = np.append(contact, curr_contact, axis=0)
   else:
      contact = curr_contact


   ## THIS IS FOR GETTING THE Z POSITION FOR EACH FOOT TO GET CONTACT
   # fr_z = (p.getLinkState(a1, 4)[0])[2]
   # fl_z = (p.getLinkState(a1, 8)[0])[2]
   # rr_z = (p.getLinkState(a1, 12)[0])[2]
   # rl_z = (p.getLinkState(a1, 16)[0])[2]
   # z_pos = np.array([fr_z, fl_z, rr_z, rl_z])
   # foot_z_pos = np.append(foot_z_pos, z_pos[np.newaxis,:], axis=0)
   base_pos = np.array(p.getBasePositionAndOrientation(a1)[0])

   fr_foot = np.array((p.getLinkState(a1, 4)[0])) - base_pos
   fl_foot = np.array((p.getLinkState(a1, 8)[0])) - base_pos
   rr_foot = np.array((p.getLinkState(a1, 12)[0])) - base_pos
   rl_foot = np.array((p.getLinkState(a1, 16)[0])) - base_pos
   foot_pos = np.concatenate([fr_foot, fl_foot, rr_foot, rl_foot])
   foot_position_robot = np.append(foot_position_robot, foot_pos[np.newaxis,:], axis=0)

foot_position_robot = foot_position_robot[1:]

   # print(curr_contact)

   # print(i)
   # print(c_FR)
   # plt.scatter(i,c_RL)
   #   print(p.getJointStates(a1,[3])[0])
   #   print('\n')

   #   plt.scatter(i, actions[i, 2] + actions[i,9])
   #   plt.scatter(i,(p.getJointStates(a1,[1])[0])[0])
   #   plt.pause(0.01)

np.save(f"foot_position_{DATA}.npy",foot_position_robot)
print(foot_position_robot.shape)
p.disconnect()

# plt.plot(range(np.shape(test)[0]), test)
plt.show()


# ## LOADING ASSETS ------------------------------------------------------------------------------------------
# planeId = p.loadURDF("plane.urdf", [0,0,0], [0,03,0,1])
# cubeStartPos = [0,0,0]
# cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# # boxId = p.loadURDF("franka_panda/panda.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=True)
# boxId = p.loadURDF("a1/a1.urdf",cubeStartPos, cubeStartOrientation)
# """
#    - loadURDF("name.urdf", position in xyz, orientation in quaternions)

#    - Each URDF is a set of links, and there is base link. We dont want the robot to fall over as it moves.
#    - p.loadURDF returns an index of each asset. This is our 0th asset.
# """
# obj_of_focus = boxId

# print(p.getNumJoints(boxId))

# for i in range(p.getNumJoints(boxId)):
#    print(p.getJointInfo(boxId, i))
#    # p.getJointInfo(asset, joint)

# jointid = 2
# jtype = p.getJointInfo(boxId, jointid)[2]
# jlower = p.getJointInfo(boxId, jointid)[8]
# jupper = p.getJointInfo(boxId, jointid)[9]

# # changing joint angles
# for step in range(500):
#    joint_2_targ = np.random.uniform(jlower, jupper)
#    joint_4_targ = np.random.uniform(jlower, jupper)
#    # VELOCITY/TORQUE/POSITION CONTROL
#    p.setJointMotorControlArray(boxId, [2], p.POSITION_CONTROL, targetPositions=[joint_2_targ])
#    p.stepSimulation()
#    time.sleep(0.01)

#    print(p.getJointStates(boxId,[2,4]))
#    # can also query the link by doing p.getLinkState(___, ___)

# ## ROBOT STEPPING ------------------------------------------------------------------------------------------
# # for step in range(1000):
# #    focus_position, _ = p.getBasePositionAndOrientation(boxId)  # returns position and orientation
# #    # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
# #    p.stepSimulation()
# #    time.sleep(0.01)
# # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# # print(cubePos,cubeOrn)

# p.disconnect()

"""
When setting up in our own environment:

class TestEnv():
   def __init__(self):
      self.state = self.init_state()
      self.step_count = 0

   def init_state(self):
      p.connect(p.GUI)
      p.resetSimulation()
      p.setAdditionalSearchPath(pybullet_data.getDataPath())
      p.setGravity(0,0,-9.8)
      self.testUid = p.loadURDF("...urdf", [0,0,0], [0,0,0,1], useFixedBase=False)
      p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])
      finger_pos = p.getLinkState(self.testUid, 9)[0]
      obx = np.array([finger_pos]).flatten()
      return obs

   def reset(self):
      p.disconnect()
      self.state = self.init_state()
      self.step_count = 0

   def step(self, action):
      self.step_count += 1
      p.setJoinMotorControlArray(self.testUid, [4], p.POSITION_CONTROL, [action])      # we can change every single joint here
      p.stepSimulation()
      finger_pos = p.getLinkState(self.testUid, 9)[0]

      if (self.step_count >= 50):
         self.reset()
         finger_pos = p.getLinkState(self.testUid, 9)[0]
         obs = np.array([finger_pos]).flatten()
         self.state = obs
         reward = -1 # arbitrary reward
         done = True
         return reward, done

      obs = np.array([finger_pos]).flatten()
      self.state = obs
      done = False
      reward = -1 # arbitrary reward

      return reward, done

      

env = TestEnv()
for step in range(500)
   action = ...
   a, b = env.step(action)
   print(env.state)
   p.stepSimulation()

"""
