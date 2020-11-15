#ifndef TDRESOURCE_H
#define TDRESOURCE_H

#include <string>
#include <iostream>
#include <time.h>
#include <fstream>
#include <string> 
#include <vector>
#include <list>
#include "map.h"
#include <iterator>     // std::back_inserter

using namespace std;

struct TDPoint
{
	unsigned int x;
	unsigned int y;
	unsigned int z;

	bool operator == (const TDPoint & obj) const //重载 “==” 操作符，函数最后的 const 别忘了，否则会报错。（详见：http://www.cnblogs.com/SZxiaochun/p/7731900.html）
	{
		return x == obj.x && y == obj.y && z==obj.z; //具体匹配条件，可以自己设定
	}

};

// 从路径中读取不可航区域
class TDResource
{
public:
	static vector<TDPoint> ReadRealTimeEnviMap(string path);
	static bool TDResource::WritePathtoTxt(list<Map::Cell*> tempPath, string path);
	//static vector<TDPoint> TDResource::vectors_intersection(vector<TDPoint> v1, vector<TDPoint> v2);
	//static vector<TDPoint> TDResource::vectors_set_union(vector<TDPoint> v1, vector<TDPoint> v2);
	//static vector<TDPoint> TDResource::deletevectors(vector<TDPoint> all, vector<TDPoint> sub);
private:

};

#endif
