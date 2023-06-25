# Import PyBullet and its data
import pybullet as p
import pybullet_data

# Let us first load the URDF (Universal Robot Description Format) file for the robot 
# we want to simulate
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
planeId = p.loadURDF("plane.urdf")
scara = p.loadURDF("URDFs/planar_2R_robot.urdf")
# nao = p.loadURDF("URDFs/nao.urdf")

# Now we need to set important parameters for our simulation. 
# These need to be done at the start of every simulation that you may want to perform
p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
p.setTimeStep(0.001) #THe lower this is, more accurate the simulation 
p.setRealTimeSimulation(10)  # we want to be faster than real time :