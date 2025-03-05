import urx
from onrobot import RG
from time import sleep


def main():
    rg.open_gripper()
    sleep(2)
    rg.close_gripper()


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.5")
    rg = RG("rg6", "192.168.1.1", "502")

    main()

    rob.close()
    rg.close_connection()
