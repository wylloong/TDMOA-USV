#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import time
from collections import deque

import GeoCommonBase
import USVMotionModelBase


class USVControl(USVMotionModelBase.USVMotionModel):
    def __init__(self, start_x, start_y, boatid):
        super(USVControl, self).__init__(boatid)
        # Reset waypoint information.
        self.tar_GeoX = 0
        self.tar_GeoY = 0
        self.tar_RotationalVelo = 0  # rotational angular velocity
        self.tar_ArrivedTime = 0
        self.tar_ArrivedRadius = 0
        self.globalTime = 0
        self.tar_lastGeoX = 0
        self.tar_lastGeoY = 0
        # The flag to denote arrival.
        self.Arrivedflag = False
        self.ArrivedNO = -1
        self.tar_GeoX = start_x
        self.tar_GeoY = start_y
        self.Current_X = start_x
        self.Current_Y = start_y
        self.lineSight = 0
        self.timeCri = False
        self.navi_Velocity = 0

        self.dTrajectory = 0
        self.dHead = 0

    def UpdateWaypoint(self, tar_x, tar_y, tar_time, tar_arrivedRadius,
                       tar_velocity):
        """Updates waypoints information."""
        self.tar_lastGeoX = self.tar_GeoX
        self.tar_lastGeoY = self.tar_GeoY
        self.tar_GeoX = tar_x
        self.tar_GeoY = tar_y
        self.tar_ArrivedTime = tar_time
        self.tar_ArrivedRadius = tar_arrivedRadius
        self.TargetSpeed = tar_velocity

    def UpdateLastWaypoint(self, last_x, last_y):
        """Updates last waypoint."""
        self.tar_lastGeoX = last_x
        self.tar_lastGeoY = last_y

    def UpdateVeolicityRefer(self, time_cricial):
        self.timeCri = time_cricial

    def UpdateMotionState(self, curr_x, curr_y, curr_heading, curr_velocity):
        """Updates motion state."""
        self.Current_X = curr_x
        self.Current_Y = curr_y
        self.BoatOutNew = curr_heading
        self.velocity = curr_velocity
        self.direction = curr_heading
        self.que = deque(
            [
                curr_velocity, curr_velocity, curr_velocity, curr_velocity,
                curr_velocity, curr_velocity, curr_velocity, curr_velocity,
                curr_velocity, curr_velocity, curr_velocity, curr_velocity,
                curr_velocity, curr_velocity, curr_velocity, curr_velocity
            ],
            maxlen=16)

    def USVCtrIteration(self, timeInterval):
        # Check whether arrive taget first.
        distance2Waypoint = GeoCommonBase.DistanceofPoints(
            self.Current_X, self.Current_Y, self.tar_GeoX, self.tar_GeoY)
        self.globalTime += timeInterval
        #print(" Distance to target is %f"%(distance2Waypoint))
        if distance2Waypoint < self.tar_ArrivedRadius:
            print("Arrived Target Waypoint!")
            self.Arrivedflag = True
            self.ArrivedNO += 1
        else:
            # Calculate the distance to trajectory.
            #print("目标航点：%f  %f 出发点： %f  %f"%(self.tar_GeoX,self.tar_GeoY,self.tar_lastGeoX,self.tar_lastGeoY))
            A, B, C = GeoCommonBase.GeneralEquation(
                self.tar_lastGeoX, self.tar_lastGeoY, self.tar_GeoX,
                self.tar_GeoY)
            dTra = math.fabs(A * self.Current_X + B * self.Current_Y + C
                             ) / math.sqrt(A**2 + B**2)
            # Get target bearing by waypoints. North is the reference.
            self.lineSight = 90 - math.atan2(
                self.tar_GeoY - self.tar_lastGeoY,
                self.tar_GeoX - self.tar_lastGeoX) * 180 / math.pi

            CurrPnt2WP = 90 - math.atan2(
                self.Current_Y - self.tar_lastGeoY,
                self.Current_X - self.tar_lastGeoX) * 180 / math.pi
            self.tar_Head = 90 - math.atan2(
                self.tar_GeoY - self.Current_Y,
                self.tar_GeoX - self.Current_X) * 180 / math.pi
            # Path Following
            if self.timeCri == False:
                self.navi_Velocity = self.TargetSpeed
            # Trajectory Tracking
            elif self.tar_ArrivedTime == 0:
                print("TT without arrivedTime information!")
                self.navi_Velocity = 15
            else:
                leftTime2Waypoint = self.tar_ArrivedTime - self.globalTime  # ETC
                # Speed is determined by time and distance.
                self.navi_Velocity = max(
                    distance2Waypoint - self.tar_ArrivedRadius, 0.1) / max(
                        leftTime2Waypoint, 0.2)  # m/s
                # The speed bound is 15m/s.
                if (math.fabs(self.navi_Velocity) > 15):
                    self.navi_Velocity = 15

            # Gets the positive/negtive.
            dTemp = CurrPnt2WP - self.tar_Head

            if dTemp < 0 and dTemp > -180:
                # Vessel on the left side of trajectory, and the expected behavior is turning right.
                dTra = -dTra
            else:
                dTra = +dTra

            # The heading difference.
            dHeading = self.tar_Head - self.direction
            # -180 < left turn < 0  0 < right turn < 180
            if (dHeading > 180):
                dHeading = dHeading - 360
            if (dHeading < -180):
                dHeading = dHeading + 360

            self.dTrajectory = dTra
            self.dHead = dHeading

            # Speed difference.
            self.TargetSpeed = self.navi_Velocity
            dVelocity = self.TargetSpeed - self.velocity
            # Updates motion parameters.
            self.UpdateInputParameters(self.navi_Velocity, self.tar_Head)
            self.MotionModelIteration(timeInterval, dHeading, dVelocity, dTra)
            # print(" Current speed is %f"%(self.velocity))


if __name__ == "__main__":
    figure = plt.figure(224)
    ax = figure.add_subplot(221)
    plt.grid(True)
    ax1 = figure.add_subplot(222)
    plt.grid(True)
    ax2 = figure.add_subplot(223)
    plt.grid(True)
    ax3 = figure.add_subplot(224)
    plt.grid(True)
    ax.axis("equal")

    # plt.ion()  # interactive mode on

    WP_x = [200, 300, 300]
    WP_y = [100, 200, 500]
    WP_time = [0, 30, 50]
    WP_radius = [30, 30, 30]
    ax.plot(WP_x, WP_y)

    usvCtr = USVControl(200, 100, 0)
    usvCtr.Arrivedflag = True
    #print(usvCtr.__dict__)
    #usvCtr.UpdateWaypoint(500,300,100,10)
    usvCtr.UpdateVeolicityRefer(True)
    TimeAxis = 0
    while True:
        if usvCtr.Arrivedflag == True:
            usvCtr.Arrivedflag = False
            if usvCtr.ArrivedNO == (len(WP_x) - 1):
                break
            # tar_x,tar_y,tar_time,tar_arrivedRadius,tar_velocity
            usvCtr.UpdateWaypoint(WP_x[usvCtr.ArrivedNO + 1],
                                  WP_y[usvCtr.ArrivedNO + 1],
                                  WP_time[usvCtr.ArrivedNO + 1],
                                  WP_radius[usvCtr.ArrivedNO + 1], 10)
        #for x in range(1000):
        TimeAxis += 1
        usvCtr.USVCtrIteration(0.1)
        # Location
        ax.scatter(usvCtr.Current_X, usvCtr.Current_Y)
        # Speed
        ax1.scatter(TimeAxis, usvCtr.velocity)
        # Bearing.
        ax2.scatter(TimeAxis, usvCtr.direction)
        # Angle Speed.
        ax3.scatter(TimeAxis, usvCtr.AngSpeedNew)
        #time.sleep(0.1)
        #plt.pause(0.01)

    plt.ioff()
    plt.show()
