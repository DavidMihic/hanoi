import urx  # https://github.com/jkur/python-urx
from time import sleep
import math3d as m3d
import json


def waitForButton(msg: str) -> None:
    input(msg)


def getPoint(point_label: str) -> m3d.PositionVector:
    waitForButton(f"Press Enter to log point: {point_label}")
    return rob.get_pos()


def logPoints() -> tuple[m3d.PositionVector]:
    p1 = getPoint("origin")
    print("p1", p1)
    rob.set_freedrive(1)

    waitForButton("Press Enter to continue...")

    p2 = getPoint("x axis")
    print("p2", p2)
    rob.set_freedrive(1)

    waitForButton("Press Enter to continue...")

    p3 = getPoint("y axis")
    print("p3", p3)
    rob.set_freedrive(1)

    waitForButton("Press Enter to lock robot movement...")

    return p1, p2, p3


def getPlane(
    origin: m3d.PositionVector,
    posXpoint: m3d.PositionVector,
    posYpoint: m3d.PositionVector,
) -> tuple[m3d.PositionVector, tuple[m3d.FreeVector]]:
    posX = posXpoint - origin
    posY = posYpoint - origin
    posZ = posX.cross(posY)

    posX.normalize()
    posY.normalize()
    posZ.normalize()

    return origin, (posX, posY, posZ)


def translate(
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


def saveCoordinateSystem(
    origin: m3d.PositionVector,
    rotation: tuple[float],
    axes: tuple[m3d.FreeVector],
    filename: str = "coordinateSystem.json",
):

    obj = {
        "origin": list(map(float, origin.array)),
        "rotation": rotation,
        "posX": list(map(float, axes[0].array)),
        "posY": list(map(float, axes[1].array)),
        "posZ": list(map(float, axes[2].array)),
    }

    with open(filename, "w") as file:
        json.dump(obj, file)


def main() -> None:
    sleep(0.2)

    waitForButton("Press Enter to unlock robot movement...")
    rob.set_freedrive(1)
    p1, p2, p3 = logPoints()
    rob.set_freedrive(0)

    origin, axes = getPlane(p1, p2, p3)

    # s ovim kutevima se treba igrati tijekom kalibracije
    # kutevi se mogu pronaci ovako:
    #   Move
    #   OBAVEZNO -> FEATURE > Base
    #   Align po z osi
    #   okreni zadnji zglob robota tako da mu alat gleda u pozitivnom smjeru x osi postava
    #   gore desno (RX, RY, RZ) su kutevi
    rotation = (0.000, 0.000, 0.000)

    originPose = m3d.Transform((*origin, *rotation))
    aboveFirstRodPose = translate(originPose, axes, x=0.075, y=0.075, z=0.2)
    aboveSecondRodPose = translate(aboveFirstRodPose, axes, y=0.2)
    aboveThirdRodPose = translate(aboveSecondRodPose, axes, y=0.2)

    sleep(2)
    waitForButton("Press Enter to move above rod 1...")

    print("moving to 1...")
    rob.set_pose(aboveFirstRodPose)

    waitForButton("Press Enter to move above rod 2...")

    print("moving to 2...")
    rob.set_pose(aboveSecondRodPose)

    waitForButton("Press Enter to move above rod 3...")

    print("moving to 3...")
    rob.set_pose(aboveThirdRodPose)

    print("done")

    print("saving coordinate system...")
    saveCoordinateSystem(origin, rotation, axes)


if __name__ == "__main__":
    print("Connecting to robot...")
    rob = urx.Robot("192.168.1.5")

    print("Starting program...\n")
    main()

    rob.close()
