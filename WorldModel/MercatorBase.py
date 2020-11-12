import math

# 经纬度转墨卡托
def lonLat2Mercator(longti,latitude):
    x = longti * 20037508.34 / 180
    y = math.log(math.tan((90 + latitude)* math.pi / 360))/(math.pi / 180)
    y = y * 20037508.34 / 180
    return x,y

# 墨卡托转经纬度
def Mercator2lonLat(mercator_x,mercator_y):
    x = mercator_x / 20037508.34 * 180;
    y = mercator_y / 20037508.34 * 180;
    y = 180 / math.pi * (2 * math.atan(math.exp(y * math.pi / 180)) - math.pi  / 2)
    return x,y

# 代码测试区域
if __name__=="__main__":
  # 12532348.271762222 2460897.9636407867
  x,y = Mercator2lonLat(12532348.271762222+3868, 2460897.9636407867+658)
  print(x,y)