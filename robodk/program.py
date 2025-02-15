import robodk
from robodk import robolink
from robodk import robomath

RDK = robolink.Robolink("localhost", port=20500)

robot = RDK.Item("UR3e")
gripper = RDK.Item("tool")

disks = [RDK.Item(f"disk_{diam}") for diam in [70, 90, 110, 130]]
# diskPositions = [disk.Pose() for disk in disks]
diskStartingPositions = [
    robodk.Pose(380, 800, 94, 90, 0, 0),
    robodk.Pose(380, 800, 64, 90, 0, 0),
    robodk.Pose(380, 800, 34, 90, 0, 0),
    robodk.Pose(380, 800, 4, 90, 0, 0),
]

homeTarget = RDK.Item("home")
rod1Above = RDK.Item("rod1_above")
rod1Pickup = RDK.Item("rod1_pickup")


def hanoi(n, src, dest, aux, instructions=[]):
    if n == 1:
        instructions.append((n, src, dest))
        return instructions

    hanoi(n - 1, src, aux, dest, instructions)
    instructions.append((n, src, dest))
    hanoi(n - 1, aux, dest, src, instructions)

    return instructions


def closeGripper(disk):
    gripper.AttachClosest(f"disk_{[70, 90, 110, 130][disk-1]}")


def openGripper():
    gripper.DetachAll()


def resetDisks():
    gripper.DetachAll()
    for i, disk in enumerate(disks):
        disk.setPose(diskStartingPositions[i])


def main():
    robot.MoveJ(homeTarget)
    resetDisks()

    n = 4
    rods = [n, 0, 0]
    for disk, src, dest in hanoi(n, 0, 1, 2):
        # go to above src rod
        aboveSrcTarget = rod1Above.Pose() * robomath.transl(-200 * src, 0, 0)
        robot.MoveL(aboveSrcTarget)

        # adjust height of gripper based on where the current disk is on the src rod
        pickupSrcTarget = rod1Pickup.Pose() * robomath.transl(
            -200 * src, -30 * rods[src] + 35, 0
        )
        robot.MoveL(pickupSrcTarget)
        closeGripper(disk)

        robot.MoveL(aboveSrcTarget)

        # go to above dest rod
        aboveDestTarget = rod1Above.Pose() * robomath.transl(-200 * dest, 0, 0)
        robot.MoveL(aboveDestTarget)

        # adjust height of gripper based on where to put the disk on dest rod
        dropoffDestTarget = rod1Pickup.Pose() * robomath.transl(
            -200 * dest, -30 * rods[dest], 0
        )
        robot.MoveL(dropoffDestTarget)
        openGripper()
        robot.MoveL(aboveDestTarget)

        rods[src] -= 1
        rods[dest] += 1


if __name__ == "__main__":
    # resetDisks()
    main()
