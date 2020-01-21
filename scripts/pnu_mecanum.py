#!/usr/bin/env python
from base_controller import MecanumBase
import rospy
import serial

if __name__ == "__main__":
    try:
        WHEEL_SEP_WIDTH = float(rospy.get_param(
            "~wheel_sep_width", "0.23"))
        WHEEL_SEP_LENGTH = float(rospy.get_param(
            "~wheel_sep_length", "0.25"))
        WHEEL_RADIUS = float(rospy.get_param("~wheel_radius", "0.1"))

        def driveFunc(arg):
            # print(arg)
            pass

        # robot = MecanumBase("base_controller", 0.25, 0.40, 0.05, driveFunc)
        robot = MecanumBase(WHEEL_RADIUS, WHEEL_SEP_WIDTH, WHEEL_SEP_LENGTH,
                            driveFunc)
        robot.spin()
    except rospy.ROSInterruptException:
        pass
