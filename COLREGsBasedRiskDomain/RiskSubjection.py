# -*- coding: utf-8 -*-

import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import GeoCommonBase as CoordinateBase

# 危险隶属度函数，障碍物周围隶属度函数随着方向和速度逐渐变小
# 改进：隶属度范围和速度大小有关
def riskSubjection(boatPnt,currPnt,boatVelo,boatOrien,evaluTime,impactFactor):
    #考虑求解半径
    evaluDistance=boatVelo*evaluTime
    #评估点与船的距离
    realDistance=CoordinateBase.Distance(boatPnt,currPnt)
    if(realDistance<=evaluDistance):
        # 评估点与船只在范围内，返回隶属函数
        # currPnt相对于boatPnt的向量
        relativeVector = CoordinateBase.Point(currPnt.x - boatPnt.x, currPnt.y - boatPnt.y)
        # currPnt与船行驶方向的夹角
        interAngle=CoordinateBase.IntersectionAngle(boatOrien,relativeVector) # 返回角度和向量的夹角
        # 方向影响量
        orienFactor = velocityDirectionFactor(interAngle,impactFactor)
        return 1-realDistance/(evaluDistance*orienFactor)
    else:
        return 0

# 速度方向影响因子
def velocityDirectionFactor(interangle,impactFactor):
    # 方向影响量
    delta = math.cos(CoordinateBase.angle2radian(interangle))
    orienFactor = 1 + impactFactor * (1 / (1 + math.e ** (-delta * 3.5))) ** 3.5 * (1 + delta) # Sigmoid函数
    return orienFactor

#求解隶属度函数值，不考虑速度方向
def subordinateFunctionWithoutOri(basePnt,currPnt,semidiameter):
    #考虑求解半径 semidiameter
    #判断是否在范围semidiameter内
    if(CoordinateBase.Distance(basePnt,currPnt)<=semidiameter):
        #在一定范围内，调用隶属函数,currPnt相对于basePnt的向量
        return 1-CoordinateBase.Distance(basePnt,currPnt)/semidiameter
    else:
        return 0

# 绕点旋转公式
def RotationWayPnt(rotIni_x,rotIni_y,edit_x,edit_y,rotaAngle):
    #点绕点旋转公式,逆时针 旋转原点rotIni，待计算点edit,逆时针旋转角度rotaAngle
    Rotaradian=rotaAngle*math.pi/180
    newX=(edit_x-rotIni_x)*math.cos(Rotaradian)-(edit_y-rotIni_y)*math.sin(Rotaradian)+rotIni_x
    newY=(edit_x-rotIni_x)*math.sin(Rotaradian)+(edit_y-rotIni_y)*math.cos(Rotaradian)+rotIni_y
    return CoordinateBase.Point(newX,newY)

if __name__=="__main__":
    boatLocation=CoordinateBase.Point(0,0)
    currLocation=CoordinateBase.Point(10,10)
    boatVelo=10
    boatOrien=45
    evaluTime=10
    impactFactor=0.7
    subjection=riskSubjection(boatLocation,currLocation,boatVelo,boatOrien,evaluTime,impactFactor)
    print (subjection)
    # 绘制等高线
    fig = plt.figure(1)  # 创建图表1
    ax = Axes3D(fig)
    X = np.arange(-150, 150, 2)
    Y = np.arange(-150, 150, 2)
    X, Y = np.meshgrid(X, Y)
    zs = np.array([riskSubjection(boatLocation,CoordinateBase.Point(x, y),boatVelo,boatOrien,evaluTime,impactFactor) for x, y in zip(np.ravel(X), np.ravel(Y))])
    Z = zs.reshape(X.shape)
    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='hot')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    contourfig = plt.figure(2)  # 创建图表2,等高线图
    coutax = contourfig.add_subplot(1, 1, 1)
    #plt.text(15, -13, "V", fontsize=15, verticalalignment="bottom", horizontalalignment="left")
    plt.contour(X, Y, Z,20)
    # coutax.set_xlabel('X Label')
    # coutax.set_ylabel('Y Label')

    ratioPlt = plt.figure(3)  # 创建图表2,等高线图
    ax2 = ratioPlt.add_subplot(3, 3, 3)
    x=0
    while x<math.pi:
        orienFactor = velocityDirectionFactor(x*180/math.pi,impactFactor)
        ax2.scatter(x, orienFactor, c='r', marker='.')  # 航路
        x+=math.pi/100
    plt.show()
