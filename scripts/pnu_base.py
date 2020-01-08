#!/usr/bin/env python
from base_controller import MecanumBase
import rospy
from cyclope import Cyclope
import time

if __name__ == "__main__":
    try:
        WHEEL_SEP_WIDTH = float(rospy.get_param(
            "~wheel_sep_width", "0.23"))
        WHEEL_SEP_LENGTH = float(rospy.get_param(
            "~wheel_sep_length", "0.25"))
        WHEEL_RADIUS = float(rospy.get_param("~wheel_radius", "0.1"))

        try:
            cyclop_front = Cyclope("/dev/pnuMecFront")

            def driveFunc(arg):
                # print(arg)
                cyclop_front.writeVel(1, arg[0])  # Front right
                rospy.sleep(0.005)
                cyclop_front.writeVel(2, -arg[1])  # Front left
                rospy.sleep(0.005)
                pass

            def odomFunc():
                drive_wheel = [0.0, 0.0, 0.0, 0.0]
                drive_wheel[0] = cyclop_front.readVel(1)/100.0  # Front right
                rospy.sleep(0.005)
                drive_wheel[1] = -cyclop_front.readVel(2)/100.0  # Front left
                rospy.sleep(0.005)
                print(drive_wheel)
                return drive_wheel

            # robot = MecanumBase("base_controller", 0.25, 0.40, 0.05, driveFunc)
            robot = MecanumBase(WHEEL_RADIUS, WHEEL_SEP_WIDTH, WHEEL_SEP_LENGTH,
                                driveFunc, odomFunc)
            robot.spin()
        except Exception as e:
            print("Cannot connect to port.")
            print("Error: " + str(e))

    except rospy.ROSInterruptException:
        pass
