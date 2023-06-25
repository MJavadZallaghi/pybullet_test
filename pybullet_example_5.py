import pybullet as p

physicsClient = p.connect(p.DIRECT)  # Connect to the physics server
robotId = p.loadURDF('URDFs\planar_2R_robot.urdf')

linkIndex = 2  # Example link index
linkState = p.getLinkState(robotId, linkIndex)

position = linkState[0]  # XYZ coordinates
orientation = linkState[1]  # Quaternion representing the orientation

print("Position: ", position)
print("orientation: ", orientation)


p.disconnect()



