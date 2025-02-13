from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox

RDK = robolink.Robolink("localhost", port=20500)

robot = RDK.Item("UR3e")
print(robot.Pose())

disks = [RDK.Item(f"disk_{diam}") for diam in [70, 90, 110, 130]]
print(disks)
