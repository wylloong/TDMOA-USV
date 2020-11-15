# 离散模型 - -  递推公式
# 初始化：初始位置 + 初始航向

from collections import deque
import math
import matplotlib.pyplot as plt
import VesselClass

class USVMotionModel(VesselClass.Vessels):
    # 初始化
    def __init__(self,boatid):
        # __init__方法的第一个参数永远是self，表示创建的实例本身
        super(USVMotionModel,self).__init__(boatid)

        # self.SpeedNew = 0       # 当前速度 - 新
        self.__SpeedOld = 0     # 当前速度 - 旧

        self.ConInNew = 0       # 控制器输入 -- 航向偏差——新
        self.__ConInOld = 0     # 控制器输入 - -航向偏差——旧
        self.ConOutNew = 0      # 控制器输出 - 新
        self.__ConOutOld = 0    # 控制器输出 - 旧
        self.AngSpeedNew=0      # 角速度 - 新
        self.__AngSpeedOld=0    # 角速度 - 旧

        self.BoatInNew = 0      # 船舶输入_新
        self.__BoatInOld = 0    # 船舶输入_旧
        # self.BoatAngleOut = 0   # 航向角（0~360）
        self.BoatOutNew=0       # 航向角（未限幅）
        self.__BoatOutOld = 0

        self.PadOut= 0          # 油门输入
        self.que=deque([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],maxlen=16) # 通过queue的构造函数可选参数maxlen来设置队列长度

        self.IteraTime=0.1 # 迭代和采样时间，默认为0.1s

    # 输入：目标速度，目标航向
    def UpdateInputParameters(self,targetSpeed,targetSpeedAngle):
        self.TargetSpeed=targetSpeed
        self.TargetHead=targetSpeedAngle

    # 运动模型实现算法
    def MotionModelIteration(self,interval,dHeading,dVelocity,dTrajectory):
        self.BoatOutNew=self.direction
        # 偏航距影响角度差输入
        # dCtrlIn=1/(1+math.exp(-math.fabs(dTrajectory/10)))*dTrajectory*(-0.85)
        # dCtrlIn = 1 / (1 + math.exp(-math.fabs(dTrajectory / 3))) * dTrajectory * (-0.08)
        dCtrlIn = dTrajectory * (-0.57) # MAH的做法
        #print("Head %f dTra %f"%(dHeading,dCtrlIn))
        # 控制器的输入是航向偏差
        # 存储控制器输入——存储两个，方便迭代
        self.__ConInOld = self.ConInNew
        # self.ConInNew = dHeading+dCtrlIn
        self.ConInNew = dHeading*0.9 + dCtrlIn

        # 控制器，零极点配置后，输出舵面并进行限幅
        # ConOut 舵角
        self.__ConOutOld=self.ConOutNew
        self.ConOutNew = 0.8667 * self.__ConOutOld + 8.9641 * self.ConInNew - 8.8707 * self.__ConInOld

        # 舵角限幅值 -35~35
        dt1 = self.ConOutNew
        if (dt1 > 35):
            dt1 = 35
        if (dt1 < -35):
            dt1 = -35

        # BoatIn： 船舶模型输入--舵角
        # 船舶模型 总体输入为舵角，输出为航向角。其中，惯性环节和积分环节分开
        self.__BoatInOld = self.BoatInNew
        self.BoatInNew = dt1  # 船舶模型输入（已限幅）
        # 惯性环节：(输入：舵角值；输出：船艏角角速度）
        # AngSpeed 角速度
        self.__AngSpeedOld=self.AngSpeedNew
        self.AngSpeedNew=0.9896 * self.__AngSpeedOld + 0.006 * self.BoatInNew + 0.006 * self.__BoatInOld
        # 积分环节：（输入：船艏角角速度；输出：船艏角，即航向角）
        self.__BoatOutOld=self.BoatOutNew
        self.BoatOutNew=self.__BoatOutOld+0.05*self.AngSpeedNew+0.05*self.__AngSpeedOld

        if (self.BoatOutNew < 0):
            self.BoatOutNew = self.BoatOutNew + 360
        if (self.BoatOutNew > 360):
            self.BoatOutNew = self.BoatOutNew- 360
        self.direction = self.BoatOutNew

        # 速度模型
        # 输入：目标速度(m/s)，延迟15至16个周期
        # 输出：速度（m/s）
        PadIn_15=0
        PadIn_16 = 0
        if(self.que.maxlen==len(self.que)):# 判断队列是否满
            # 迭代，存储16个目标速度，FIFO
            PadIn_16=self.que.pop() # 删除并返回最右侧的元素
            PadIn_15 =self.que.pop() # 删除并返回最右侧的元素
            self.que.append(PadIn_15) # 在队列的末尾添加一个元素
        self.que.appendleft(self.TargetSpeed) # 油门输出,置于队列最左侧

        # 速度迭代：输入为油门，输出为速度
        self.__SpeedOld=self.velocity
        self.velocity=0.92 * self.__SpeedOld+0.04 * PadIn_15+0.04 * PadIn_16

        # 位置信息X，Y 未加速度控制，固定步长StepLength，每次根据角度分别进行X, Y的累加
        self.__last_X=self.Current_X
        self.__last_Y=self.Current_Y
        self.Current_X= self.__last_X +self.velocity * math.sin(math.pi*self.BoatOutNew/180)*interval    # 步长是1
        self.Current_Y = self.__last_Y + self.velocity * math.cos(math.pi*self.BoatOutNew/180)*interval  # 步长是1
        # 输出：位置信息，航向，速度
        # print("位置 X= %f ,Y=%f ,方向= %f ,速度=%f 航向偏差=%f 角速度= %f 舵角=%f 目标速度=%f 目标航向=%f"
        #       % (self.Curr_X,self.Curr_Y,self.BoatOutNew,self.SpeedNew,self.ConInNew,self.AngSpeedNew,self.BoatInNew,self.TargetSpeed,self.TargetHead))
        return self.Current_X,self.Current_Y,self.BoatOutNew,self.velocity,self.direction

if __name__=="__main__":
    figure=plt.figure()
    ax=figure.add_subplot(111)
    ax.axis("equal")  # 设置图像显示的时候XY轴比例
    plt.grid(True)  # 添加网格
    plt.ion()  # interactive mode on
    # 运动模型初始化
    usvMotionModel = USVMotionModel()
    usvMotionModel.Ini_X = 0
    usvMotionModel.Ini_Y = 0
   
    usvMotionModel.UpdateInputParameters(10, 0)
    for x in range(300):
        Curr_X,Curr_Y,boatAngleOut,speedNew,angveoli=usvMotionModel.MotionModelIteration()
        ax.scatter(Curr_X,Curr_Y)
        plt.pause(0.01)
    print("位置 X= %f ,Y=%f ,方向= %f ,速度=%f" % (Curr_X, Curr_Y, boatAngleOut, speedNew))
   
    usvMotionModel.UpdateInputParameters(20, 100)
    for x in range(600):
        Curr_X,Curr_Y,boatAngleOut,speedNew,angveoli=usvMotionModel.MotionModelIteration()
        ax.scatter(Curr_X,Curr_Y)
        plt.pause(0.01)
    print("位置 X= %f ,Y=%f ,方向= %f ,速度=%f" % (Curr_X, Curr_Y, boatAngleOut, speedNew))
   
    usvMotionModel.UpdateInputParameters(8, -100)
    for x in range(600):
        Curr_X,Curr_Y,boatAngleOut,speedNew,angveoli=usvMotionModel.MotionModelIteration()
        ax.scatter(Curr_X,Curr_Y)
        plt.pause(0.01)
    print("位置 X= %f ,Y=%f ,方向= %f ,速度=%f" % (Curr_X, Curr_Y, boatAngleOut, speedNew))
   
    plt.ioff() # 关闭交互模式
   
    # 界面显示
    plt.show()
