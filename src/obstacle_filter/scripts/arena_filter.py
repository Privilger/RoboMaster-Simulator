class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

# calc |p1 p2| X |p1 p|

def GetCross(p1, p2, p):
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y)


def IsPointInA(p, t):
	p1 = Point(1.40-t, 2.40+t)
	p2 = Point(1.40-t, 1.40-t)
	p3 = Point(1.65+t, 1.40-t)
	p4 = Point(1.65+t, 2.40+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def IsPointInB(p, t):
	p1 = Point(1.20-t, 4.00+t)
	p2 = Point(1.20-t, 3.75-t)
	p3 = Point(2.20+t, 3.75-t)
	p4 = Point(2.20+t, 4.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0
 

def IsPointInC(p, t):
	p1 = Point(3.25-t, 1.00+t)
	p2 = Point(3.25-t, 0.00-t)
	p3 = Point(3.50+t, 0.00-t)
	p4 = Point(3.50+t, 1.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >=  0
 

def IsPointInD(p, t):
	p1 = Point(3.50-t, 2.625+t)
	p2 = Point(3.50-t, 2.375-t)
	p3 = Point(4.50+t, 2.375-t)
	p4 = Point(4.50+t, 2.625+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0
 

def IsPointInE(p, t):
	p1 = Point(4.50-t, 5.00+t)
	p2 = Point(4.50-t, 4.00-t)
	p3 = Point(4.75+t, 4.00-t)
	p4 = Point(4.75+t, 5.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0
 

def IsPointInF(p, t):
	p1 = Point(5.80-t, 1.25+t)
	p2 = Point(5.80-t, 1.00-t)
	p3 = Point(6.80+t, 1.00-t)
	p4 = Point(6.80+t, 1.25+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0
 

def IsPointInG(p, t):
	p1 = Point(6.35-t, 3.60+t)
	p2 = Point(6.35-t, 2.60-t)
	p3 = Point(6.60+t, 2.60-t)
	p4 = Point(6.60+t, 3.60+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0
 

def IsPointInEdge1(p, t):
	p1 = Point(-10.00-t, 5.00+t)
	p2 = Point(-10.00-t, 0.00-t)
	p3 = Point( 0.00+t, 0.00-t)
	p4 = Point( 0.00+t, 5.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >=  0
 

def IsPointInEdge2(p, t):
	p1 = Point(-10.00-t, 60.00+t)
	p2 = Point(-10.00-t, 5.00-t)
	p3 = Point( 90.00+t, 5.00-t)
	p4 = Point( 90.00+t, 60.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >=  0
 

def IsPointInEdge3(p, t):
	p1 = Point( 8.00-t, 5.00+t)
	p2 = Point( 8.00-t, 0.00-t)
	p3 = Point( 90.00+t, 0.00-t)
	p4 = Point( 90.00+t, 5.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >=  0
 

def IsPointInEdge4(p, t):
	p1 = Point(-10.00-t,  0.00+t)
	p2 = Point(-10.00-t, -10.00-t)
	p3 = Point( 90.00+t, -10.00-t)
	p4 = Point( 90.00+t,  0.00+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >=  0
 


def IsPointInArena(p, t):
	return IsPointInA(p, t) or IsPointInB(p, t) or IsPointInC(p, t) or IsPointInD(p, t) or IsPointInE(p, t) or IsPointInF(p, t) or IsPointInG(p, t) or IsPointInEdge1(p, t) or IsPointInEdge2(p, t) or IsPointInEdge3(p, t) or IsPointInEdge4(p, t)
 

