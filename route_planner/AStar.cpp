#include <math.h>
#include <fstream>
#include <iostream>

#include "AStar.h"

void Astar::InitAstar(const std::string &filename)
{
	/*    pointVec points;
	std::ifstream in(filename);
	while(in.eof())
	{
	double x,y;
	in>>x>>y;
	points.push_back({x,y});
	}
	waypoints(points);
	in.close();
	*/
}


double Astar::calcG(Point *temp_start, Point *point)
{
	double extraG = sqrt(pow((point->x - temp_start->x), 2) + pow((point->y - temp_start->y), 2));

	double parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	double result = parentG + extraG;
	if (result < 1e-8){
		result = 0;
	}
	return result;
}

double Astar::calcH(Point *point, Point *end)
{
	//ŷ����ξ���
	double result = sqrt(pow((end->x - point->x), 2) + pow((end->y - point->y), 2));
	if (result < 1e-8){
		result = 0;
	}
	return result;
}

double Astar::calcF(Point *point)
{
	return point->G + point->H;
}

Point *Astar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
			if (point->F<resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //�������,��������һ���ڵ㣬�������
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		//std::cout<<curPoint->x<<" "<<curPoint->y<<std::endl;
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			if (!isInList(openList, target))
			{	
				target->parent = curPoint;
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
			else
			{
				double tempG = calcG(curPoint, target);
				if (tempG<target->G)
				{
					target->parent = curPoint;
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			Point *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
		}
	}

	return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//����·�������û�ҵ�·�������ؿ�����
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	// �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
	openList.clear();
	closeList.clear();

	return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for (auto p : list)
		if (p->x == point->x&&p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
	if (isInList(closeList, target))
	{
		return 0;
	}
	return 1;
	/*	if (target->x<0 || target->x>maze.size() - 1
	|| target->y<0 || target->y>maze[0].size() - 1
	|| maze[target->x][target->y] == 1
	|| target->x == point->x&&target->y == point->y
	|| isInList(closeList, target)) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����false
	return false;
	else
	{
	if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //��б�ǿ���
	return true;
	else
	{
	//б�Խ�Ҫ�ж��Ƿ��ס
	if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
	return true;
	else
	return isIgnoreCorner;
	}
	}
	*/
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;

	pointVec kdsurroundingpoints;
	point_t thispoint;
	thispoint.push_back(point->x);
	thispoint.push_back(point->y);

	kdsurroundingpoints = waypoints.neighborhood_points(thispoint, searching_rad);

	for (auto p : kdsurroundingpoints)
	{
		Point *sp = new Point(p[0], p[1]);
		if (isCanreach(point, sp, isIgnoreCorner))
			surroundPoints.push_back(sp);
	}
	//for (int x = point->x - 1; x <= point->x + 1; x++)
	//for (int y = point->y - 1; y <= point->y + 1; y++)
	//if (isCanreach(point, new Point(x, y), isIgnoreCorner))
	//surroundPoints.push_back(new Point(x, y));
	return surroundPoints;
}
