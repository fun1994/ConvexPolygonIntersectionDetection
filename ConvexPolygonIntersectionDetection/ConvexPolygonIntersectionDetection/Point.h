#pragma once

class Point {
public:
	double x;
	double y;
	bool extreme;
	bool operator==(Point& p) {
		return x == p.x && y == p.y;
	}
};
