import urx
import urx.urscript
import urx.ursecmon  # https://github.com/jkur/python-urx
from onrobot import RG
from time import sleep
import math3d as m3d

# m/s, m/s^2
FAST_VEL = 0.75
SLOW_VEL = 0.3
ACCEL = 0.01

# position [m], rotation [rad]
HOME_TARGET = m3d.Transform((-0.495, 0.145, 0.416, -2.068, -2.365, 0.000))
ABOVE_ROD_0_TARGET = m3d.Transform((-0.502, 0.345, 0.259, -2.068, -2.365, 0.000))

# length in meters
ROD_DISTANCE = 0.202  # 20 cm plus error
DISK_HEIGHT = 0.03
DISK_DIAMETERS = [0.13, 0.11, 0.9, 0.7]
NUM_DISKS = 4


def openGripper() -> None:
    rg.open_gripper()

    while True:
        sleep(0.5)
        if not rg.get_status()[0]:
            break


def closeGripper() -> None:
    rg.move_gripper(500, 250)

    while True:
        sleep(0.5)
        if not rg.get_status()[0]:
            break


def translate(
    pose: m3d.Transform, x: float = 0.0, y: float = 0.0, z: float = 0.0
) -> m3d.Transform:
    newPose = m3d.Transform(pose)
    newPose.pos.x += x
    newPose.pos.y += y
    newPose.pos.z += z

    return newPose


def hanoi(
    n: int, src: int, dest: int, aux: int, instructions: list[tuple] = []
) -> list[tuple]:
    if n == 1:
        instructions.append((n, src, dest))
        return instructions

    hanoi(n - 1, src, aux, dest, instructions)
    instructions.append((n, src, dest))
    hanoi(n - 1, aux, dest, src, instructions)

    return instructions


def executeHanoi() -> None:
    openGripper()
    rods = [NUM_DISKS, 0, 0]
    for disk, src, dest in hanoi(NUM_DISKS, 0, 1, 2):
        # go above src rod
        aboveSrcTarget = translate(ABOVE_ROD_0_TARGET, y=-ROD_DISTANCE * src)
        rob.set_pose(aboveSrcTarget, vel=FAST_VEL, acc=ACCEL)

        # adjust height of gripper based on where the current disk is on src rod
        pickupSrcTarget = translate(
            aboveSrcTarget, z=-0.217 + DISK_HEIGHT * (rods[src] - 1)
        )
        rob.set_pose(pickupSrcTarget, vel=SLOW_VEL, acc=ACCEL)
        closeGripper()
        rob.set_pose(aboveSrcTarget, vel=SLOW_VEL, acc=ACCEL)

        # go above dest rod
        aboveDestTarget = translate(ABOVE_ROD_0_TARGET, y=-ROD_DISTANCE * dest)
        rob.set_pose(aboveDestTarget, vel=FAST_VEL, acc=ACCEL)

        # adjust height of gripper based on where to put the disk on dest rod
        dropoffDestTarget = translate(
            aboveDestTarget, z=-0.210 + DISK_HEIGHT * rods[dest]
        )
        rob.set_pose(dropoffDestTarget, vel=SLOW_VEL, acc=ACCEL)
        openGripper()
        rob.set_pose(aboveDestTarget, vel=SLOW_VEL, acc=ACCEL)

        rods[src] -= 1
        rods[dest] += 1


def main() -> None:
    rob.set_payload(1.2)
    sleep(0.2)  # give time for robot to process setup commands

    rob.set_pose(HOME_TARGET, vel=FAST_VEL, acc=ACCEL, command="movej")

    executeHanoi()

    rob.set_pose(HOME_TARGET, vel=SLOW_VEL, acc=ACCEL, command="movej")


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    main()

    rob.close()
    rg.close_connection()
