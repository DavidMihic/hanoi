import urx  # https://github.com/jkur/python-urx
import urx.urscript
import urx.ursecmon
from onrobot import RG
from time import sleep
import math3d as m3d

# offset from tip of sleeve to TCP, in meters
SLEEVE_OFFSET_X = 0.025
SLEEVE_OFFSET_Y = 0.025


def getPoint():
    input("Teach the robot a point...")
    return rob.get_pos()


def main() -> None:
    rob.set_payload(1.2)
    sleep(0.2)

    sleep(2)

    rob.set_freedrive(1)

    p1 = getPoint()
    p2 = getPoint()
    p3 = getPoint()

    rob.set_freedrive(0)

    print(p1)
    print(p2)
    print(p3)


if __name__ == "__main__":
    print("Connecting to robot and gripper...")
    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    print("Starting program...\n")
    main()

    rob.close()
    rg.close_connection()
