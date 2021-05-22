#include "planner.h"
#include "math.h"
#include "map.h"
#include "TDResource.h"
#include <iostream>
#include <time.h>
#include <fstream>
#include <string> 
#include <vector>
#include <cstdio> 
#include <windows.h> 
#include <cmath>
#include <iostream>

using namespace std;

#define PI 3.14159265

double angleDiff(double start_x, double start_y, double target_x, double target_y, double curr_x, double curr_y)
{
	double diff_x = target_x - start_x+0.01;
	double diff_y = target_y - start_y;
	double slope = 360+(90-atan2(diff_y,diff_x)*180/PI); 
	if (slope >= 360)
		slope -= 360;
	diff_x = target_x - curr_x+0.01;
	diff_y = target_y - curr_y;
	double curr_slope = 360 + (90 - atan2(diff_y, diff_x) * 180 / PI);
	if (curr_slope > 360)
		curr_slope -= 360;
	double angleDiff = curr_slope - slope;
	if (abs(angleDiff) <= 90)
		angleDiff = abs(angleDiff);
	else if (abs(angleDiff) > 90 && abs(angleDiff) < 270)
		angleDiff = abs(180 - abs(angleDiff));
	else
		angleDiff = 360 - abs(angleDiff);

	return angleDiff;
}

int main()
{
	string FILENAME = "E:\\MultiEnv.txt";
	const double knotmile = 1852;  //m

	const double USVVelocity = 16;  //kn/h
	const double unitTime = 10; //s
	const double unitLength = USVVelocity*knotmile / 3600* unitTime;
	//const int img_rows = int(16000 / (16* 1852 / 3600 * 45) + 1);
	//const int img_cols = int(16000 / (16 * 1852 / 3600 * 45) + 1);
	//const int img_tiers = int(max(img_rows, img_cols)*1.3);

	const int img_rows = 70; // 70
	const int img_cols = 60;
	const int img_tiers = 90;

	printf(" �������� %d \n", img_rows*img_cols*img_tiers);
	// Make the map
	Map* _map = new Map(img_rows, img_cols, img_tiers);
	// Make planner
	vector<TDPoint> config = TDResource::ReadRealTimeEnviMap("E:\\MultiOBSConfig.txt");
	Map::Cell* start = (*_map)(config[0].x, config[0].y, config[0].z);
	Map::Cell* target = (*_map)(config[1].x, config[1].y, config[1].z);
	Planner* _planner = new Planner(_map, start, target);

	double start_x = (start->x() + 0.5) * unitLength;
	double start_y = (start->y() + 0.5) * unitLength;
	double target_x = (target->x() + 0.5) * unitLength;
	double target_y = (target->y() + 0.5) * unitLength;
	//printf(" ��ʼ�� (%f, %f) Ŀ��� (%f, %f) \n", start_x, start_y, target_x, target_y);

	printf("  ���ϰ���ֲ�·���滮��ʼ�� \n");
	double angletoSTLine = 0;
	double curr_x = 0;
	double curr_y = 0;
	// Build map
	
	for (int k = 0; k < img_tiers; k++)
	{
		for (int j = 0; j < img_cols; j++)
		{
			for (int i = 0; i < img_rows; i++)
			{
				// ���ݺ�Ŀ�꺽·�ĽǶȲ�,���û�������
				curr_y = (i + 0.5)*unitLength;
				curr_x = (j + 0.5)*unitLength;
				angletoSTLine = angleDiff(start_x, start_y, target_x, target_y, curr_x, curr_y);
				//printf(" (%f, %f) %f \n", curr_x, curr_y, angletoSTLine);
				// double v = 1+ weighttoSlope*;
				double v = exp(abs(sin(angletoSTLine / 180 * PI)) * 0.2);
				// ��Ŀ��㿿��
				double Edgedis = sqrt(pow(target->x() - i, 2) + pow(target->y() - j, 2) + pow(target->z() - k, 2)) + 0.000001;  //0.00001����ֹ��ĸΪ0
				double Linedis = sqrt(pow(target->x() - i, 2) + pow(target->y() - j, 2));
				double sinValue = Linedis / Edgedis;
				// double ratio = 1 - 0.8*cos(asin(sinValue));
				/*double ratio = 1 + sinValue;*/
				double ratio= exp(sinValue * abs(sin(angletoSTLine / 180 * PI)));

				/*if (k == 0)
					printf("%d %d %d %f\n", i, j, k, v);*/
				// (*_map)(i, j, k)->cost = v*ratio;
				(*_map)(i, j, k)->cost = ratio;
				if((ratio)<1.0)
					printf(" ע�⣺ȨֵС�ڵ���2������ ");
			}
		}
	}




	clock_t startTime, endTime;
	list<Map::Cell*> path_planned;
	Map::Cell* u;

	// ������ѵ��ʵ����python��αʵʱ����
	while (true)
	{
		Sleep(200);  //1000Ϊ1s �ʵ���ʱ����ֹ���ٲ�ѯ
		fstream _file;
		_file.open(FILENAME, ios::in);
		if (!_file)
		{
			// �ļ�������
			continue;
		}
		else
		{
			_file.close();  //�ر��ļ��������޷�ɾ��
			// ���ڣ���ȡ��ע��ɾ��txt
			// ���ò��ɺ�����
			printf("������̽�⵽�ֲ������仯��·�����¹滮�� \n");
			startTime = clock();  //��ʼ��ʱ
			// ��ȡUSV��ʼ��(��ǰλ��)��Ŀ���
			vector<TDPoint> config = TDResource::ReadRealTimeEnviMap("E:\\MultiOBSConfig.txt");
			Map::Cell* start = (*_map)(config[0].x, config[0].y, config[0].z);
			Map::Cell* target = (*_map)(config[1].x, config[1].y, config[1].z);
			_planner->start(start);
			u = _planner->start();
			printf(" USV λ��( %d ,%d ,%d )����㣬��⵽�ֲ������仯��·�����¹滮��\n", u->x(), u->y(), u->z());

			// ��ȡ���ɺ��е㴦��
			vector<TDPoint> unwalks = TDResource::ReadRealTimeEnviMap(FILENAME);
			vector<TDPoint>::iterator iter;
			printf(" ����������ĸ����� %d \n", unwalks.size());
			for (iter = unwalks.begin(); iter != unwalks.end(); iter++)
			{
				u = (*_map)((*iter).x, (*iter).y, (*iter).z);
				double costTemp = u->cost;
				if (costTemp == DBL_MAX)
				{
					continue;
				}
				else
				{
					// printf(" ����( %d ,%d ,%d )�ڵ� ", u->x(), u->y(), u->z());
					// �����Ĳ��ɺ���
					_planner->update(u, DBL_MAX);
				}		
			}
			
			// ��ȡ��ע��ɾ��txt
			Sleep(1000);  //1000Ϊ1s �ʵ���ʱ����ֹ���ٲ�ѯ
			if (remove(FILENAME.c_str()) == 0) //"ɾ���ɹ�"
			{
				printf("����������ϣ�ɾ�������ļ��ɹ� \n");
			}
			else
			{
				printf("��������error��ɾ�������ļ�ʧ�� \n");
			}
			endTime = clock();
			printf(" ��������ʱ�� runtime: %f s \n", (long double)(endTime - startTime) / CLOCKS_PER_SEC);
		}
		
		startTime = clock();  //��ʼ��ʱ
		printf(" D* Lite�㷨��ʼ��������·�� \n");
		if (!_planner->replan())
		{
			// û���ҵ��ɺ��е�·��
			cout << "Fail" << endl;
			continue;
		}
		else
		{
			if (_planner->path().size()>1)
			{
				cout << "Succ" << endl;
				path_planned = _planner->path();
			}
			endTime = clock();
			list<Map::Cell*> tempPath = path_planned;
			TDResource::WritePathtoTxt(tempPath, "E:\\localPath.txt");

			while (!tempPath.empty())
			{
				Map::Cell* cell = tempPath.front();
				tempPath.pop_front();
				printf("( %d ,%d ,%d ) -> ", cell->x(), cell->y(), cell->z());
			}
			printf("\n");
			printf("runtime: %f s \n", (long double)(endTime - startTime) / CLOCKS_PER_SEC);
			printf("\n");

			int expanedNum = 0;
			// ��ʾ������չ��Ŀ
			for (int i = 0; i < img_rows; i++)
			{
				for (int j = 0; j < img_cols; j++)
				{
					for (int k = 0; k < img_tiers; k++)
					{
						if ((*_map)(i, j, k)->expandedStatus)
						{
							expanedNum++;
						}
					}
				}
			}
			printf("������չ��Ŀ: %d \n", expanedNum);

		}
	}
	

	// ģ�� USV ��ʻ����������
	int exitTime = 0;
	while (!path_planned.empty() && _planner->start() != _planner->goal() && exitTime<1)
	{
		_planner->start(path_planned.front());
		u = _planner->start();
		path_planned.pop_front();
		printf(" USV ʻ��( %d ,%d ,%d )�����\n", u->x(), u->y(), u->z());
		exitTime++;
	}

	getchar();
	// Push start position
	// _real_widget->path_traversed.push_back(_planner->start());
}
