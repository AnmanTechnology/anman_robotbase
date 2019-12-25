#!/usr/bin/env python

from cyclope import Cyclope


if __name__ == "__main__":
    print("Test drive, Cyclope board")
    try:
        cyclope = Cyclope('/dev/ttyUSB0')
        cyclope.setPID(1, 1, 0, 0)
        # cyclope.setPID(2, 1, 0, 0)
        # cyclope.setPID(3, 1, 0, 0)
        # cyclope.setPID(4, 1, 0, 0)
    except Exception as e:
        print("Cannot connect to port.")
        print("Error: " + str(e))
