#!/usr/bin/env python3

import collections
import math


class Vessel():
    """Class Vessel defines a model for any vessels."""

    def __init__(self, boatid):
        self.id = boatid

        # Initiation.
        self.Initial_X = 0
        self.Initial_Y = 0
        self.Current_X = 0
        self.Current_Y = 0
        self.last_X = 0
        self.last_Y = 0
        self.TargetHead = 0  # Target heading for the vessel.
        self.TargetSpeed = 0  # Target speed for the vessel.
        self.velocity = 0
        self.direction = 0
        self.scale = 200  # The size of the vessel in meters.
        self.boatType = 0  # The type of the vessel.

    def ReStartVesselState(self, x, y, velocity, direction):
        """Resets the state of the vessel."""
        self.Initial_X = x
        self.Initial_Y = y
        self.Current_X = x
        self.Current_Y = y
        # Set last position to current position.
        self.last_X = x
        self.last_Y = y  #
        self.velocity = velocity
        self.direction = direction

    def UpdateLocation(self, x, y):
        """Updates the position."""
        self.Current_X = x
        self.Current_Y = y

    def UpdateKineticChara(self, velocity, direction):
        """Updates the motion state of the vessel."""
        self.velocity = velocity
        self.direction = direction

    def UpdateBoatStaticChara(self, scale, boatType):
        self.scale = scale
        self.boatType = boatType

    def BoatMotionItera(self, iteraTime):
        """Computes the postion by the iterative time."""
        self.Current_X += self.velocity * math.sin(
            self.direction / 180 * math.pi) * iteraTime
        self.Current_Y += self.velocity * math.cos(
            self.direction / 180 * math.pi) * iteraTime
        return self.Current_X, self.Current_Y


if (__name__ == "__main__"):
    vessel = Vessel(1)
    print(vessel.id)
