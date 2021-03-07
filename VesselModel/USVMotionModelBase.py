#!/usr/bin/env python3

from collections import deque
import math
import matplotlib.pyplot as plt

import VesselClass


class USVMotionModel(VesselClass.Vessel):
    def __init__(self, boatid):
        super(USVMotionModel, self).__init__(boatid)
        # Vessel speed
        self.__SpeedOld = 0
        # Controller input
        self.ConInNew = 0
        self.__ConInOld = 0
        # Contronller output
        self.ConOutNew = 0
        self.__ConOutOld = 0
        # Angel speed
        self.AngSpeedNew = 0
        self.__AngSpeedOld = 0
        # Boat input
        self.BoatInNew = 0
        self.__BoatInOld = 0
        # Boat bearing output without limitation.
        self.BoatOutNew = 0
        self.__BoatOutOld = 0
        # Pad inut.
        self.PadOut = 0
        self.que = deque(
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], maxlen=16)
        # Iteration and sampling interval time in seconds.
        self.IteraTime = 0.1

    def UpdateInputParameters(self, targetSpeed, targetSpeedAngle):
        """Updates target speed and target angle."""
        self.TargetSpeed = targetSpeed
        self.TargetHead = targetSpeedAngle

    def MotionModelIteration(self, interval, dHeading, dVelocity, dTrajectory):
        """Motion model core."""
        self.BoatOutNew = self.direction
        # The distance to trajectory influences angle difference.
        # dCtrlIn=1/(1+math.exp(-math.fabs(dTrajectory/10)))*dTrajectory*(-0.85)
        # dCtrlIn = 1 / (1 + math.exp(-math.fabs(dTrajectory / 3))) * dTrajectory * (-0.08)
        dCtrlIn = dTrajectory * (-0.57)  # From MAH
        # print("Head %f dTra %f"%(dHeading,dCtrlIn))
        # Controller input is heading difference.
        self.__ConInOld = self.ConInNew
        self.ConInNew = dHeading * 0.9 + dCtrlIn

        # Controller output.
        self.__ConOutOld = self.ConOutNew
        self.ConOutNew = 0.8667 * self.__ConOutOld + 8.9641 * self.ConInNew - 8.8707 * self.__ConInOld

        # Limit output within [-35,~35]
        dt1 = self.ConOutNew
        if (dt1 > 35):
            dt1 = 35
        if (dt1 < -35):
            dt1 = -35

        # Vessel model input.
        self.__BoatInOld = self.BoatInNew
        self.BoatInNew = dt1
        # First, Inertial element: input is rudder angle, and output is bearing angle speed.
        self.__AngSpeedOld = self.AngSpeedNew
        self.AngSpeedNew = 0.9896 * self.__AngSpeedOld + 0.006 * self.BoatInNew + 0.006 * self.__BoatInOld
        # Second, Integral element: Input is bearing angle speed, and output is bearing.
        self.__BoatOutOld = self.BoatOutNew
        self.BoatOutNew = self.__BoatOutOld + 0.05 * self.AngSpeedNew + 0.05 * self.__AngSpeedOld

        if (self.BoatOutNew < 0):
            self.BoatOutNew = self.BoatOutNew + 360
        if (self.BoatOutNew > 360):
            self.BoatOutNew = self.BoatOutNew - 360
        self.direction = self.BoatOutNew

        # Speed model with delay up to 15 cycles.
        # Input the target speed and output current speed in m/s.
        PadIn_15 = 0
        PadIn_16 = 0
        # Cache historical target speed in FIFO que.
        if (self.que.maxlen == len(self.que)):
            PadIn_16 = self.que.pop()
            PadIn_15 = self.que.pop()
            self.que.append(PadIn_15)
        self.que.appendleft(self.TargetSpeed)
        # Speed interation: Input is pad and output is speed.
        self.__SpeedOld = self.velocity
        self.velocity = 0.92 * self.__SpeedOld + 0.04 * PadIn_15 + 0.04 * PadIn_16

        # Updates current location by steplength.
        self.__last_X = self.Current_X
        self.__last_Y = self.Current_Y
        self.Current_X = self.__last_X + self.velocity * math.sin(
            math.pi * self.BoatOutNew / 180) * interval
        self.Current_Y = self.__last_Y + self.velocity * math.cos(
            math.pi * self.BoatOutNew / 180) * interval
        # 输出：位置信息，航向，速度
        # print("位置 X= %f ,Y=%f ,方向= %f ,速度=%f 航向偏差=%f 角速度= %f 舵角=%f 目标速度=%f 目标航向=%f"
        #       % (self.Curr_X,self.Curr_Y,self.BoatOutNew,self.SpeedNew,self.ConInNew,self.AngSpeedNew,self.BoatInNew,self.TargetSpeed,self.TargetHead))
        return self.Current_X, self.Current_Y, self.BoatOutNew, self.velocity, self.direction


if __name__ == "__main__":
    figure = plt.figure()
    ax = figure.add_subplot(111)
    ax.axis("equal")
    plt.grid(True)
    plt.ion()  # interactive mode on
    usvMotionModel = USVMotionModel(1)
    usvMotionModel.Ini_X = 0
    usvMotionModel.Ini_Y = 0

    usvMotionModel.UpdateInputParameters(10, 0)

    plt.ioff()  # interactive mode off
    plt.show()
