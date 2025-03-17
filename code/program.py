import urx
import urx.urscript
import urx.ursecmon  # https://github.com/jkur/python-urx
from onrobot import RG
from time import sleep
import math3d as m3d

# m/s, m/s^2
FAST_VEL = 0.75
SLOW_VEL = 0.3
ACCEL = 0.02

# position [m], rotation [rad]
HOME_TARGET = m3d.Transform((-0.495, 0.145, 0.416, -2.068, -2.365, 0.000))
ABOVE_ROD_0_TARGET = m3d.Transform((-0.502, 0.345, 0.225, -2.068, -2.365, 0.000))

# length in meters
ROD_DISTANCE = 0.201  # 20 cm plus error
DISK_HEIGHT = 0.03
DISK_DIAMETERS = [700, 900, 1100, 1300]  # 1/10mm, like onrobot rg likes it
NUM_DISKS = 5


def moveGripper(width: int, force: int = 250) -> None:
    rg.move_gripper(width, force)

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
    rods = [NUM_DISKS, 0, 0]
    for disk, src, dest in hanoi(NUM_DISKS, 0, 1, 2):
        xAdjustment = -0.01 if disk >= 4 else 0  # for larger disks move closer

        aboveSrcTarget = translate(
            ABOVE_ROD_0_TARGET, y=-ROD_DISTANCE * src, x=xAdjustment
        )
        pickupSrcTarget = translate(
            aboveSrcTarget, z=-0.183 + DISK_HEIGHT * (rods[src] - 1)
        )
        aboveDestTarget = translate(
            ABOVE_ROD_0_TARGET, y=-ROD_DISTANCE * dest, x=xAdjustment
        )
        dropoffDestTarget = translate(
            aboveDestTarget, z=-0.177 + DISK_HEIGHT * rods[dest]
        )

        rob.set_pose(aboveSrcTarget, vel=FAST_VEL, acc=ACCEL)
        moveGripper(DISK_DIAMETERS[disk - 1] + 100)  # open extra
        rob.set_pose(pickupSrcTarget, vel=SLOW_VEL, acc=ACCEL)
        moveGripper(
            500
        )  # don't close all the way so in case of an error a rod isn't grabbed
        rob.set_pose(aboveSrcTarget, vel=SLOW_VEL, acc=ACCEL)

        rob.set_pose(aboveDestTarget, vel=FAST_VEL, acc=ACCEL)
        rob.set_pose(dropoffDestTarget, vel=SLOW_VEL, acc=ACCEL)
        moveGripper(DISK_DIAMETERS[disk - 1] + 100)  # open extra
        rob.set_pose(aboveDestTarget, vel=SLOW_VEL, acc=ACCEL)

        rods[src] -= 1
        rods[dest] += 1


def main() -> None:
    rob.set_payload(1.2)
    sleep(0.2)  # give time for robot to process setup commands

    rob.set_pose(HOME_TARGET, vel=FAST_VEL, acc=ACCEL, command="movej")
    executeHanoi()
    rob.set_pose(HOME_TARGET, vel=FAST_VEL, acc=ACCEL, command="movej")


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    main()

    rob.close()
    rg.close_connection()
