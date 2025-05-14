# pip install urx
import urx  # https://github.com/jkur/python-urx
from onrobot import RG
from time import sleep
import math3d as m3d
import json

# m/s, m/s^2
# FAST_VEL = 0.75 #TODO uncommentat, povecat po potrebi
# SLOW_VEL = 0.3
# ACCEL = 0.02
FAST_VEL = 0.01
SLOW_VEL = 0.01
ACCEL = 0.01

homeTarget = None
aboveRod0Target = None

# length in meters
ROD_DISTANCE = 0.201  # 20 cm plus error #TODO podesiti
DISK_HEIGHT = 0.03
DISK_DIAMETERS = [600, 700, 900, 1100, 1300]  # 1/10mm, like onrobot rg likes it
NUM_DISKS = 5


def loadCoordinateSystem(
    filename: str = "coordinateSystem.json",
) -> tuple[m3d.Transform, m3d.FreeVector]:
    with open(filename, "r") as file:
        obj = json.loads(file.read())

    originPose = m3d.Transform((*obj["origin"], *obj["rotation"]))
    posX = m3d.FreeVector(obj["posX"])
    posY = m3d.FreeVector(obj["posY"])
    posZ = m3d.FreeVector(obj["posZ"])

    return originPose, (posX, posY, posZ)


def calculateInitialTargets(
    originPose: m3d.Transform, axes: tuple[m3d.FreeVector]
) -> None:
    global homeTarget, aboveRod0Target
    homeTarget = translateWRTAxes(originPose, axes, x=0.075, y=0.275, z=0.35)
    aboveRod0Target = translateWRTAxes(
        originPose, axes, x=0.075 + 0.034, y=0.075, z=0.2
    )


def moveGripper(width: int, force: int = 250) -> None:
    rg.move_gripper(width, force)

    while True:
        sleep(0.5)
        if not rg.get_status()[0]:
            break


def translateWRTAxes(
    pose: m3d.Transform,
    axes: tuple[m3d.FreeVector],
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> m3d.Transform:
    translBase = x * axes[0] + y * axes[1] + z * axes[2]

    newPose = m3d.Transform(pose)
    newPose.pos += translBase

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


def executeHanoi(axes: tuple[m3d.FreeVector]) -> None:
    moveGripper(1400)
    rods = [NUM_DISKS, 0, 0]
    instructions = hanoi(NUM_DISKS, 0, 1, 2) + [(NUM_DISKS, None, None)]
    for i, (disk, src, dest) in enumerate(instructions):
        if i == len(instructions) - 1:
            return

        # for larger disks move closer
        xAdjustment = -0.01 if DISK_DIAMETERS[disk - 1] >= 1300 else 0  # TODO podesiti
        nextDisk = instructions[i + 1][0]
        gripperOpenWidth = (
            max(DISK_DIAMETERS[disk - 1], DISK_DIAMETERS[nextDisk - 1]) + 100
        )

        aboveSrcTarget = translateWRTAxes(
            aboveRod0Target, axes, x=xAdjustment, y=-ROD_DISTANCE * src
        )
        pickupSrcTarget = translateWRTAxes(
            aboveSrcTarget, z=-0.183 + DISK_HEIGHT * (rods[src] - 1)  # TODO podesiti
        )
        aboveDestTarget = translateWRTAxes(
            aboveRod0Target, axes, y=-ROD_DISTANCE * dest, x=xAdjustment
        )
        dropoffDestTarget = translateWRTAxes(
            aboveRod0Target, axes, z=-0.177 + DISK_HEIGHT * rods[dest]  # TODO podesiti
        )

        rob.set_pose(aboveSrcTarget, vel=FAST_VEL, acc=ACCEL)
        rob.set_pose(pickupSrcTarget, vel=SLOW_VEL, acc=ACCEL)
        # don't close all the way so in case of an error a rod isn't grabbed
        moveGripper(500)
        rob.set_pose(aboveSrcTarget, vel=SLOW_VEL, acc=ACCEL)

        rob.set_pose(aboveDestTarget, vel=FAST_VEL, acc=ACCEL)
        rob.set_pose(dropoffDestTarget, vel=SLOW_VEL, acc=ACCEL)
        moveGripper(gripperOpenWidth)
        rob.set_pose(aboveDestTarget, vel=SLOW_VEL, acc=ACCEL)

        rods[src] -= 1
        rods[dest] += 1


def main() -> None:
    rob.set_digital_out(0, 1)  # pneumatic hold of tool
    sleep(0.2)  # give time for robot to process setup commands

    originPose, axes = loadCoordinateSystem()
    calculateInitialTargets(originPose, axes)

    rob.set_pose(homeTarget, vel=FAST_VEL, acc=ACCEL)
    executeHanoi(axes)
    rob.set_pose(homeTarget, vel=FAST_VEL, acc=ACCEL)


if __name__ == "__main__":
    assert NUM_DISKS == len(DISK_DIAMETERS)

    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    main()

    rob.close()
    rg.close_connection()
