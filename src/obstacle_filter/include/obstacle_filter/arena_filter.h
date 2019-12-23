//判断一个点是否在矩形内部
#include<cstdio>
#include<iostream>

struct Point
{
	float x;
	float y;
	Point(float x,float y)
	{
		this->x = x;
		this->y = y;
	}
};
// 计算 |p1 p2| X |p1 p|
float GetCross(Point& p1, Point& p2,Point& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}

bool IsPointInA(Point& p, float t)
{
	Point p1(1.40-t, 2.40+t);
	Point p2(1.40-t, 1.40-t);
	Point p3(1.65+t, 1.40-t);
	Point p4(1.65+t, 2.40+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB(Point& p, float t)
{
	Point p1(1.20-t, 4.00+t);
	Point p2(1.20-t, 3.75-t);
	Point p3(2.20+t, 3.75-t);
	Point p4(2.20+t, 4.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInC(Point& p, float t)
{
	Point p1(3.25-t, 1.00+t);
	Point p2(3.25-t, 0.00-t);
	Point p3(3.50+t, 0.00-t);
	Point p4(3.50+t, 1.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInD(Point& p, float t)
{
	Point p1(3.50-t, 2.625+t);
	Point p2(3.50-t, 2.375-t);
	Point p3(4.50+t, 2.375-t);
	Point p4(4.50+t, 2.625+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInE(Point& p, float t)
{
	Point p1(4.50-t, 5.00+t);
	Point p2(4.50-t, 4.00-t);
	Point p3(4.75+t, 4.00-t);
	Point p4(4.75+t, 5.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInF(Point& p, float t)
{
	Point p1(5.80-t, 1.25+t);
	Point p2(5.80-t, 1.00-t);
	Point p3(6.80+t, 1.00-t);
	Point p4(6.80+t, 1.25+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInG(Point& p, float t)
{
	Point p1(6.35-t, 3.60+t);
	Point p2(6.35-t, 2.60-t);
	Point p3(6.60+t, 2.60-t);
	Point p4(6.60+t, 3.60+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge1(Point& p, float t)
{
	Point p1(-10.00-t, 5.00+t);
	Point p2(-10.00-t, 0.00-t);
	Point p3( 0.00+t, 0.00-t);
	Point p4( 0.00+t, 5.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge2(Point& p, float t)
{
	Point p1(-10.00-t, 60.00+t);
	Point p2(-10.00-t, 5.00-t);
	Point p3( 90.00+t, 5.00-t);
	Point p4( 90.00+t, 60.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge3(Point& p, float t)
{
	Point p1( 8.00-t, 5.00+t);
	Point p2( 8.00-t, 0.00-t);
	Point p3( 90.00+t, 0.00-t);
	Point p4( 90.00+t, 5.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge4(Point& p, float t)
{
	Point p1(-10.00-t,  0.00+t);
	Point p2(-10.00-t, -10.00-t);
	Point p3( 90.00+t, -10.00-t);
	Point p4( 90.00+t,  0.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}


bool IsPointInAerna(Point& p, float t)
{
	return IsPointInA(p, t) || IsPointInB(p, t) || IsPointInC(p, t) || IsPointInD(p, t) || IsPointInE(p, t) || IsPointInF(p, t) || IsPointInG(p, t) || IsPointInEdge1(p, t) || IsPointInEdge2(p, t) || IsPointInEdge3(p, t) || IsPointInEdge4(p, t);
}

