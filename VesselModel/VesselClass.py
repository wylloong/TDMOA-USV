# -*- coding: utf-8 -*-
"""
Created on Sat May  6 07:21:32 2017

@author: wyl
"""

import collections
import math

#Point = collections.namedtuple("Point", ["Pnt_x", "Pnt_y"])
#BoatBase = collections.namedtuple("BoatBase", ["BoatLocation", "BoatDirection"])  # 船参数定义

class Vessels(object):
    # pass
    def __init__(self, boatid):  # __init__方法的第一个参数永远是self，表示创建的实例本身
        self.id = boatid
        # 初始位置
        self.Initial_X=0
        self.Initial_Y = 0
        # 当前位置
        self.Current_X = 0
        self.Current_Y = 0
        # 上一次位置
        self.last_X = 0
        self.last_Y = 0
        # 目标速度
        self.TargetHead = 0  # 目标航向
        self.TargetSpeed = 0  # 目标速度
        # 当前速度和方向
        self.velocity = 0
        self.direction = 0
        self.scale = 200  # 船的大小
        self.boatType = 0  # 船的类型

    # 初始化状态
    def ReStartVesselState(self,x, y,velocity, direction):
        # 初始位置
        self.Initial_X = x
        self.Initial_Y = y
        # 当前位置
        self.Current_X = x
        self.Current_Y = y
        # 上一次位置
        self.last_X = x  # 上一次位置x
        self.last_Y = y  # 上一次位置y
        self.velocity = velocity
        self.direction = direction

    # 更新位置
    def UpdateLocation(self, x, y):
        self.Current_X = x
        self.Current_Y = y

    # 更新运动状态
    def UpdateKineticChara(self, velocity, direction):
        self.velocity = velocity
        self.direction = direction

    # 定义船只状态
    def UpdateBoatStaticChara(self, scale, boatType):
        self.scale = scale  # 船的大小
        self.boatType = boatType  # 船的类型

    # 定义船只迭代运算
    def BoatMotionItera(self,iteraTime):
        self.Current_X += self.velocity * math.sin(self.direction / 180 * math.pi) * iteraTime  # 当前位置x
        self.Current_Y += self.velocity * math.cos(self.direction / 180 * math.pi) * iteraTime  # 当前位置y
        return self.Current_X, self.Current_Y

if (__name__ == "__main__"):
    vess = Vessels(0)  # 创建实例
    print(Vessels)