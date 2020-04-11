class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

# calc |p1 p2| X |p1 p|

def GetCross(p1, p2, p):
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y)


def isPointIn1(p, t):
	p1 = Point(0.00-t, 5.10+t)
	p2 = Point(0.00-t, 2.55-t)
	p3 = Point(2.66+t, 2.55-t)
	p4 = Point(2.66+t, 5.10+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def isPointIn2(p, t):
	p1 = Point(2.66-t, 5.10+t)
	p2 = Point(2.66-t, 2.55-t)
	p3 = Point(5.33+t, 2.55-t)
	p4 = Point(5.33+t, 5.10+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def isPointIn3(p, t):
	p1 = Point(5.33-t, 5.10+t)
	p2 = Point(5.33-t, 2.55-t)
	p3 = Point(8.10+t, 2.55-t)
	p4 = Point(8.10+t, 5.10+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def isPointIn4(p, t):
	p1 = Point(2.66-t, 2.55+t)
	p2 = Point(2.66-t, 0.00-t)
	p3 = Point(5.33+t, 0.00-t)
	p4 = Point(5.33+t, 2.55+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def isPointIn5(p, t):
	p1 = Point(0.00-t, 2.55+t)
	p2 = Point(0.00-t, 0.00-t)
	p3 = Point(2.66+t, 0.00-t)
	p4 = Point(2.66+t, 2.55+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def isPointIn6(p, t):
	p1 = Point(5.33-t, 2.55+t)
	p2 = Point(5.33-t, 0.00-t)
	p3 = Point(8.10+t, 0.00-t)
	p4 = Point(8.10+t, 2.55+t)
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 and GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0

def belongToDistrict(x, y):
	t = 0
	if isPointIn1(Point(x, y), t):
		return 1
	elif isPointIn2(Point(x, y), t):
		return 2
	elif isPointIn3(Point(x, y), t):
		return 3
	elif isPointIn3(Point(x, y), t):
		return 3
	elif isPointIn4(Point(x, y), t):
		return 4
	elif isPointIn5(Point(x, y), t):
		return 5
	elif isPointIn6(Point(x, y), t):
		return 6
	else:
		return 7

def setGoal(goalID):
	if goalID == 1:
		return 1.7, 4
	elif goalID == 2:
		return 4.2, 3.4
	elif goalID == 3:
		return 7.4, 3.5
	elif goalID == 4:
		return 4.12, 1.77
	elif goalID == 5:
		return 1, 1.5
	elif goalID == 6:
		return 6.4, 1.4

