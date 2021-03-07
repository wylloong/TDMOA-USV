#!/usr/bin/env python3

import collections
import math

Point = collections.namedtuple("Point", ["x", "y"])


def RotationWayPnt(rotIni_x, rotIni_y, edit_x, edit_y, rotaAngle):
    #点绕点旋转公式,逆时针 旋转原点rotIni，待计算点edit,逆时针旋转角度rotaAngle
    Rotaradian = rotaAngle * math.pi / 180
    newX = (edit_x - rotIni_x) * math.cos(Rotaradian) - (
        edit_y - rotIni_y) * math.sin(Rotaradian) + rotIni_x
    newY = (edit_x - rotIni_x) * math.sin(Rotaradian) + (
        edit_y - rotIni_y) * math.cos(Rotaradian) + rotIni_y
    return Point(newX, newY)


#沿某一方向指定距离的点的增量
def ExtensionCordPoint(extenAngle, distance):
    ExtenAngle = extenAngle * math.pi / 180
    addx = distance * math.sin(ExtenAngle)
    addy = distance * math.cos(ExtenAngle)
    return addx, addy


def DistanceofPoints(first_x, first_y, second_x, second_y):
    return math.sqrt(
        math.pow((second_x - first_x), 2) + math.pow((second_y - first_y), 2))


def GeneralEquation(first_x, first_y, second_x, second_y):
    # Ax+By+C=0
    A = second_y - first_y
    B = first_x - second_x
    C = second_x * first_y - first_x * second_y
    return A, B, C


def NorthAngle(startPnt, terminaPnt):
    '''
    实现已知两个点坐标求与正北夹角度数,顺时针增加
    '''
    yDelta = terminaPnt.Pnt_y - startPnt.Pnt_y  # y的增量
    xDelta = terminaPnt.Pnt_x - startPnt.Pnt_x  # x的增量
    # math.atan2(y,x) 返回弧度，以横坐标正半轴为0，逆时针增加
    angle = -math.atan2(yDelta, xDelta) + math.pi / 2  # 范围是-90~270
    if angle < 0:
        angle += 2 * math.pi  # 转换范围是0~360
    return angle * 180 / math.pi


def Distance(basePnt, currPnt):
    '''
    求两点的距离
    basePnt:点的坐标
    currPnt:点的坐标
    '''
    return math.sqrt(
        math.pow((basePnt.x - currPnt.x), 2) + math.pow(
            (basePnt.y - currPnt.y), 2))


# 得到可行角度的交集
def GetLinesUnion(linelist):
    if linelist:
        # 空list本身等同于 False
        startTemp = linelist[0].Line_start
        endTemp = linelist[0].Line_end
        for line in linelist:
            if line.Line_start <= endTemp and line.Line_end >= startTemp:
                # 两条线段有交集
                startTemp = max(line.Line_start, startTemp)
                endTemp = min(line.Line_end, endTemp)
            else:
                # 无解
                return 0, 0
        if startTemp <= endTemp:
            return startTemp, endTemp
        else:
            return 0, 0
    else:
        return 0, 0


# 角度与弧度的转换
def angle2radian(angle):
    return angle * math.pi / 180


def radian2angle(radian):
    return radian * 180 / math.pi


#求两个向量的夹角cos值 向量的点乘
def VectorsCos(basePnt, currPnt):
    vectorproduct = basePnt.x * currPnt.x + basePnt.y * currPnt.y
    iniPnt = Point(0, 0)  #方便求解的人造零点
    absproduct = Distance(iniPnt, basePnt) * Distance(iniPnt, currPnt)
    if (absproduct == 0):
        # 零向量是长度为0的向量,且其方向是任意的
        return 0  #此处默认为90度
    cosVector = vectorproduct / absproduct
    if (cosVector > 1.0 or cosVector < -1.0):
        # 分别取整数部分和小数部分
        cosVector = math.modf(cosVector)[1]
    return cosVector


#根据一个方向和一个向量求两者夹角
def IntersectionAngle(angle, Vector):
    # 角度化为向量，单位为1
    scale = 1
    ang_x = scale * math.sin(angle2radian(angle))
    ang_y = scale * math.cos(angle2radian(angle))
    basePnt = Point(ang_x, ang_y)
    #math.acos() 求x的反余弦(结果是弧度),超出范围返回0
    return radian2angle(math.acos(VectorsCos(basePnt, Vector)))


def GetIntersectPointofLines(x1, y1, x2, y2, x3, y3, x4, y4):
    A1, B1, C1 = GeneralEquation(x1, y1, x2, y2)
    A2, B2, C2 = GeneralEquation(x3, y3, x4, y4)
    m = A1 * B2 - A2 * B1
    if m == 0:
        print("无交点")
    else:
        x = (C2 * B1 - C1 * B2) / m
        y = (C1 * A2 - C2 * A1) / m
    return x, y


if __name__ == "__main__":
    x, y = GetIntersectPointofLines(10, 20, 100, 200, 50, 20, 20, 100)
    print(x, y)
