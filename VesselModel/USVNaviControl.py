'''
Copyright(c) 2017, waylon
All rights reserved.
Distributed under the BSD license.
'''

import USVMotionModelBase
import GeoCommonBase
import matplotlib.pyplot as plt
import time
import math
from collections import deque # 引入队列，先进先出

class USVControl(USVMotionModelBase.USVMotionModel):

    def __init__(self,start_x,start_y,boatid):
        # 如果子类继承了多个父类，它只需要使用一次super函数就可以
        super(USVControl,self).__init__(boatid)
        # 航路点信息
        self.tar_GeoX = 0
        self.tar_GeoY = 0
        self.tar_RotationalVelo=0    # rotational angular velocity 旋转角速度
        self.tar_ArrivedTime=0
        self.tar_ArrivedRadius=0

        self.globalTime=0
        self.tar_lastGeoX = 0
        self.tar_lastGeoY = 0
        self.Arrivedflag=False      # 抵达标志位
        self.ArrivedNO=-1
        self.tar_GeoX=start_x
        self.tar_GeoY=start_y
        self.Current_X=start_x
        self.Current_Y=start_y
        self.lineSight=0
        self.timeCri=False
        self.navi_Velocity = 0

        self.dTrajectory=0          # 后文会用到
        self.dHead=0                # 后文会用到

    # 更新航路点信息
    def UpdateWaypoint(self,tar_x,tar_y,tar_time,tar_arrivedRadius,tar_velocity):
        self.tar_lastGeoX = self.tar_GeoX
        self.tar_lastGeoY = self.tar_GeoY
        self.tar_GeoX = tar_x
        self.tar_GeoY = tar_y
        self.tar_ArrivedTime = tar_time
        self.tar_ArrivedRadius = tar_arrivedRadius
        self.TargetSpeed=tar_velocity

    # 更新上一个航路点信息
    def UpdateLastWaypoint(self,last_x,last_y):
        self.tar_lastGeoX = last_x
        self.tar_lastGeoY = last_y

    def UpdateVeolicityRefer(self,time_cricial):
        self.timeCri=time_cricial

    # 更新运动状态
    def UpdateMotionState(self,curr_x,curr_y,curr_heading,curr_velocity):
        self.Current_X=curr_x
        self.Current_Y=curr_y
        self.BoatOutNew=curr_heading
        self.velocity=curr_velocity
        self.direction=curr_heading
        self.que=deque([curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,
                        curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,
                        curr_velocity,curr_velocity],maxlen=16)

    def USVCtrIteration(self,timeInterval):
        # 判断是否到达目标航点
        distance2Waypoint=GeoCommonBase.DistanceofPoints(self.Current_X,self.Current_Y,self.tar_GeoX,self.tar_GeoY)
        self.globalTime+=timeInterval
        #print("距离目标 %f"%(distance2Waypoint))
        if distance2Waypoint<self.tar_ArrivedRadius:
            print("Arrived Target Waypoint!")
            self.Arrivedflag=True
            self.ArrivedNO+=1
        else:
            # 偏航距计算
            #print("目标航点：%f  %f 出发点： %f  %f"%(self.tar_GeoX,self.tar_GeoY,self.tar_lastGeoX,self.tar_lastGeoY))
            A,B,C=GeoCommonBase.GeneralEquation(self.tar_lastGeoX,self.tar_lastGeoY,self.tar_GeoX,self.tar_GeoY)
            dTra=math.fabs(A*self.Current_X + B*self.Current_Y+C)/math.sqrt(A**2+B**2)
            # 根据WP确定航线角度
            self.lineSight = 90-math.atan2(self.tar_GeoY - self.tar_lastGeoY,
                                            self.tar_GeoX - self.tar_lastGeoX) * 180 / math.pi  # 正北方向

            CurrPnt2WP = 90-math.atan2(self.Current_Y - self.tar_lastGeoY,
                                            self.Current_X - self.tar_lastGeoX) * 180 / math.pi  # 正北方向
            # 目标参数
            self.tar_Head=90-math.atan2(self.tar_GeoY - self.Current_Y,
                                            self.tar_GeoX - self.Current_X) * 180 / math.pi  # 正北方向
            # 航路没有时间约束，Path Following
            if self.timeCri == False:
                self.navi_Velocity=self.TargetSpeed
            # Trajectory Tracking
            elif self.tar_ArrivedTime==0:
                # 异常情形
                print("TT without arrivedTime information!")
                self.navi_Velocity=15
            else:
                leftTime2Waypoint = self.tar_ArrivedTime - self.globalTime  # 到达目标点的预估时间
                # 时间和距离决定速度
                self.navi_Velocity = max(distance2Waypoint-self.tar_ArrivedRadius, 0.1)/ max(leftTime2Waypoint,0.2)  # m/s
                # 速度限幅，这一部分最好不要
                # if self.navi_Velocity>10.6:
                #     self.navi_Velocity = 10.5
                # elif self.navi_Velocity < 9.4:
                #     self.navi_Velocity = 9.5
                # else:
                #     self.navi_Velocity = self.TargetSpeed

                # 目标速度限幅,最高速度15m/s
                if (math.fabs(self.navi_Velocity) > 15):
                    self.navi_Velocity = 15

            # 偏航距符号
            dTemp=CurrPnt2WP-self.tar_Head

            if dTemp<0 and dTemp>-180:
                # 船在航线左侧，期望右转，偏航距符号为-
                dTra=-dTra
            else:
                # 船在航线右侧，期望左转，偏航距符号为+
                dTra=+dTra

            # 航向偏差
            dHeading = self.tar_Head - self.direction  # 航向偏差
            # -180< 左转 <0  0< 右转 <180
            if (dHeading > 180):
                dHeading = dHeading - 360
            if (dHeading < -180):
                dHeading = dHeading + 360

            # 后文会用到
            self.dTrajectory=dTra
            self.dHead=dHeading

            # 速度偏差
            self.TargetSpeed=self.navi_Velocity
            dVelocity = self.TargetSpeed - self.velocity
            # 更新船只在该位置朝向目标点的方向，以及根据时间约束的速度
            self.UpdateInputParameters(self.navi_Velocity, self.tar_Head)  # 更新目标运动参数
            self.MotionModelIteration(timeInterval,dHeading,dVelocity,dTra)
            # print("当前速度 %f"%(self.velocity))

if __name__=="__main__":
    figure = plt.figure(224)
    ax = figure.add_subplot(221)
    plt.grid(True)  # 添加网格
    ax1 = figure.add_subplot(222)
    plt.grid(True)  # 添加网格
    ax2 = figure.add_subplot(223)
    plt.grid(True)  # 添加网格
    ax3 = figure.add_subplot(224)
    plt.grid(True)  # 添加网格
    ax.axis("equal")  # 设置图像显示的时候XY轴比例

    # plt.ion()  # interactive mode on

    # 绘制目标点
    WP_x=[200,300,300]
    WP_y=[100,200,500]
    WP_time=[0,30,50]
    WP_radius=[30,30,30]
    ax.plot(WP_x,WP_y)

    usvCtr=USVControl(200,100,0)
    usvCtr.Arrivedflag = True
    #print(usvCtr.__dict__)
    #usvCtr.UpdateWaypoint(500,300,100,10)
    usvCtr.UpdateVeolicityRefer(True)
    TimeAxis=0
    while True:
        if usvCtr.Arrivedflag==True:
            usvCtr.Arrivedflag=False
            if usvCtr.ArrivedNO==(len(WP_x)-1):
                break
            # tar_x,tar_y,tar_time,tar_arrivedRadius,tar_velocity
            usvCtr.UpdateWaypoint(WP_x[usvCtr.ArrivedNO+1],WP_y[usvCtr.ArrivedNO+1],WP_time[usvCtr.ArrivedNO+1],WP_radius[usvCtr.ArrivedNO+1],10)
        #for x in range(1000):
        TimeAxis+=1
        usvCtr.USVCtrIteration(0.1)
        # 位置
        ax.scatter(usvCtr.Curr_X, usvCtr.Curr_Y)
        # 速度
        ax1.scatter(TimeAxis,usvCtr.velocity)
        # 航向角
        ax2.scatter(TimeAxis, usvCtr.direction)
        # 角速度
        ax3.scatter(TimeAxis, usvCtr.AngSpeedNew)
        #time.sleep(0.1)
        #plt.pause(0.01)

    plt.ioff()  # 关闭交互模式
    # 界面显示
    plt.show()