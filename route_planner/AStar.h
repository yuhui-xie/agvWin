#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <list>
#include <string>
#include "KDTree.hpp"


struct Point
{
	double x, y; //������
	double F, G, H; //F=G+H
	Point *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
	Point() : x(0.0), y(0.0), F(0.0), G(0.0), H(0.0), parent(NULL){}
	Point(double _x, double _y) :x(_x), y(_y), F(0.0), G(0.0), H(0.0), parent(NULL){}
};

class Astar
{
public:
	Astar() = default;
	Astar(const double &_rad, const pointVec &waypts) :searching_rad(_rad), waypoints(waypts){}
	void InitAstar(const std::string &filename);
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);

public:
	KDTree waypoints;
	double searching_rad = 0.2;

private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
	std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //�ж�ĳ���Ƿ����������һ���ж�
	Point *isInList(const std::list<Point *> &list, const Point *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
	Point *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
	//����FGHֵ
	double calcG(Point *temp_start, Point *point);
	double calcH(Point *point, Point *end);
	double calcF(Point *point);
private:
	std::list<Point *> openList;  //�����б�
	std::list<Point *> closeList; //�ر��б�
};

#endif
