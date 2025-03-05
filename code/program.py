import urx
import urx.urscript
import urx.ursecmon  # https://github.com/jkur/python-urx
from onrobot import RG
from time import sleep

FAST_VEL = 0.75
SLOW_VEL = 0.3

HOME_TARGET = [
    3.1737236976623535,
    -1.0330191415599366,
    -1.7965435981750488,
    -1.8829833469786585,
    1.5768370628356934,
    0.4197049140930176,
]
ABOVE_ROD_1_TARGET = [
    2.5289292335510254,
    -1.404278115635254,
    -2.0054268836975098,
    -1.3028734487346192,
    1.5768728256225586,
    -0.17434579530824834,
]


def openGripper():
    rg.open_gripper()

    while True:
        sleep(0.5)
        if not rg.get_status()[0]:
            break


def closeGripper():
    rg.close_gripper()

    while True:
        sleep(0.5)
        if not rg.get_status()[0]:
            break


def main():
    rob.set_gravity([0.0, 0.0, 9.82])
    rob.set_payload(1.2)

    # print(rob.getj())

    rob.movej(HOME_TARGET, vel=FAST_VEL)
    rob.movej(ABOVE_ROD_1_TARGET, vel=FAST_VEL)
    rob.movel([0, 0, -0.23, 0, 0, 0], vel=SLOW_VEL, relative=True)


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    main()

    rob.close()
    rg.close_connection()
