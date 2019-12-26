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
	Point p1(1.60-t, 1.04+t);
	Point p2(1.60-t, 0.09-t);
	Point p3(1.85+t, 0.09-t);
	Point p4(1.85+t, 1.04+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB(Point& p, float t)
{
    Point p1(3.58-t, 1.29+t);
    Point p2(3.58-t, 1.05-t);
    Point p3(4.58+t, 1.05-t);
    Point p4(4.58+t, 1.29+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInC(Point& p, float t)
{
    Point p1(7.21-t, 1.28+t);
    Point p2(7.21-t, 1.06-t);
    Point p3(8.15+t, 1.06-t);
    Point p4(8.15+t, 1.28+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInD(Point& p, float t)
{
	Point p1(1.62-t, 2.74+t);
	Point p2(1.62-t, 2.48-t);
	Point p3(2.40+t, 2.48-t);
	Point p4(2.40+t, 2.74+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInE(Point& p, float t)
{
	Point p1(3.93-t, 2.60+t);
	Point p2(4.17-t, 2.40-t);
	Point p3(4.38+t, 2.60-t);
	Point p4(4.17+t, 2.80+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInF(Point& p, float t)
{
	Point p1(5.92-t, 2.71+t);
	Point p2(5.92-t, 2.44-t);
	Point p3(6.68+t, 2.44-t);
	Point p4(6.68+t, 2.71+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInG(Point& p, float t)
{
	Point p1(0.16-t, 4.17+t);
	Point p2(0.16-t, 3.93-t);
	Point p3(1.08+t, 3.93-t);
	Point p4(1.08+t, 4.17+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInH(Point& p, float t)
{
    Point p1(3.60-t, 4.13+t);
    Point p2(3.60-t, 3.90-t);
    Point p3(4.60+t, 3.90-t);
    Point p4(4.60+t, 4.13+t);
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInI(Point& p, float t)
{
    Point p1(6.47-t, 5.10+t);
    Point p2(6.47-t, 4.12-t);
    Point p3(6.72+t, 4.12-t);
    Point p4(6.72+t, 5.10+t);
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
	return IsPointInA(p, t) || IsPointInB(p, t) || IsPointInC(p, t) || IsPointInD(p, t) || IsPointInE(p, t) || IsPointInF(p, t) || IsPointInG(p, t) || IsPointInH(p, t) || IsPointInI(p, t) || IsPointInEdge1(p, t) || IsPointInEdge2(p, t) || IsPointInEdge3(p, t) || IsPointInEdge4(p, t);
}

