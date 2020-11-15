#include "TDResource.h"
#include <string>
#include <iostream>
#include <time.h>
#include <fstream>
#include <vector>
#include "map.h"
#include <list>
#include <algorithm> //sort函数、交并补函数 
#include <iterator>     // std::back_inserter

using namespace std;

vector<TDPoint> TDResource::ReadRealTimeEnviMap(string path)
{
	// 读取地图信息
	vector<TDPoint> unwalkableCells;
	ifstream in(path);

	string line;
	std::string::size_type pos1;
	std::string::size_type pos2;
	while (getline(in, line))
	{
		int size = line.size();

		pos1 = line.find(" ", 0);
		pos2 = line.find(",", pos1);
		string strx = line.substr(pos1 + 1, pos2 - pos1 - 1);

		pos1 = line.find(" ", pos2 + 5);
		pos2 = line.find(",", pos1);
		string stry = line.substr(pos1 + 1, pos2 - pos1 - 1);

		pos1 = line.find(" ", pos2 + 5);
		pos2 = line.find("}", pos1);
		string strz = line.substr(pos1 + 1, pos2 - pos1 - 1);
		int x = atoi(strx.c_str());
		int y = atoi(stry.c_str());
		int z = atoi(strz.c_str());

		// 保存不可航点
		TDPoint point = { x,y,z };
		unwalkableCells.push_back(point);
	}
	in.close();
	return unwalkableCells;
}

// 规划路径写入txt文件中
bool TDResource::WritePathtoTxt(list<Map::Cell*> tempPath,string path)
{
	ofstream in;
	in.open(path,ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
	while (!tempPath.empty())
	{
		Map::Cell* cell = tempPath.front();
		tempPath.pop_front();
		//{"x": 29, "y" : 39, "z" : 68}
		in << "{\"x\": " << cell->x() << ", \"y\" : " << cell->y() << ", \"z\" : " << cell->z() <<"}"<<"\n";
	}
	in.close();
	return true;
}

/*
//两个vector求交集  
vector<TDPoint> TDResource::vectors_intersection(vector<TDPoint> v1, vector<TDPoint> v2) 
{
	vector<TDPoint> v;
	sort(v1.begin(), v1.end());
	sort(v2.begin(), v2.end());
	//set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));//求交集   
	return v;
}

//两个vector求并集  
vector<TDPoint> TDResource::vectors_set_union(vector<TDPoint> v1, vector<TDPoint> v2) 
{
	vector<TDPoint> v;
	sort(v1.begin(), v1.end());
	sort(v2.begin(), v2.end());
	//set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));//求交集   
	return v;
}

// 删除all中的sub子集
vector<TDPoint> TDResource::deletevectors(vector<TDPoint> all, vector<TDPoint> sub)
{
	vector<TDPoint> v;
	vector<TDPoint>::iterator iter;
	vector<TDPoint>::iterator it;
	for (iter = all.begin(); iter != all.end(); iter++)
	{
		it = find(sub.begin(), sub.end(), *iter);

		if (it == sub.end())
		{
			//sub中不存在值，则删除
			v.push_back(*iter);
		}
	}
	return v;
}
*/