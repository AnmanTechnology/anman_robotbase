#!/usr/bin/env python

from cyclope import Cyclope
import time

if __name__ == "__main__":
    print("Test drive, Cyclope board")

    try:

        cyclope = Cyclope('/dev/ttyACM0')
        tt = 0
        while True:
            cyclope.writeVel(2, 5.0)
            time.sleep(0.005)
            tt = cyclope.readVel(2)/100.0
            # print("{0:.2f}".format(tt))
            time.sleep(0.005)

        # cyclope.setPID(1, 1, 0, 0)
        # cyclope.setPID(2, 1, 0, 0)
        # cyclope.setPID(3, 1, 0, 0)
        # cyclope.setPID(4, 1, 0, 0)
    except Exception as e:
        print("Cannot connect to port.")
        print("Error: " + str(e))
