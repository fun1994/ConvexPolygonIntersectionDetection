#pragma once
#include <vector>
#include "Segment.h"

class ConvexPolygonIntersectionDetection {
	double area2(Point& p, Point& q, Point& r) {
		return p.x * q.y - p.y * q.x + q.x * r.y - q.y * r.x + r.x * p.y - r.y * p.x;
	}
	bool toLeft(Point& p, Point& q, Point& r) {
		return area2(p, q, r) > 0;
	}
	bool intersect(Segment& s1, Segment& s2) {
		return toLeft(s1.first, s1.second, s2.first) != toLeft(s1.first, s1.second, s2.second) && toLeft(s2.first, s2.second, s1.first) != toLeft(s2.first, s2.second, s1.second);
	}
	Point intersectionPoint(Segment& s1, Segment& s2) {
		double a1 = area2(s1.first, s2.first, s2.second);
		double a2 = area2(s1.second, s2.first, s2.second);
		Point p;
		p.x = a2 / (a2 - a1) * s1.first.x + a1 / (a1 - a2) * s1.second.x;
		p.y = a2 / (a2 - a1) * s1.first.y + a1 / (a1 - a2) * s1.second.y;
		return p;
	}
	bool inConvexPolygon(std::vector<Point>& P, Point& p) {
		for (int i = 0; i < P.size(); i++) {
			if (!toLeft(P[i], P[i == P.size() - 1 ? 0 : i + 1], p)) {
				return false;
			}
		}
		return true;
	}
	bool inConvexPolygon(std::vector<Point>& P1, std::vector<Point>& P2) {
		for (int i = 0; i < P2.size(); i++) {
			if (!inConvexPolygon(P1, P2[i])) {
				return false;
			}
		}
		return true;
	}
	int lowest(std::vector<Point>& P) {
		int left = 0;
		int right = P.size() - 1;
		if (P[0].y < P[P.size() - 1].y) {
			while (left < right) {
				int mid = (left + right) / 2;
				if (P[mid].y < P[mid + 1].y) {
					right = mid;
				}
				else {
					if (P[mid].y > P[0].y) {
						right = mid - 1;
					}
					else {
						left = mid + 1;
					}
				}
			}
		}
		else {
			while (left < right) {
				int mid = (left + right) / 2;
				if (P[mid].y > P[mid + 1].y) {
					left = mid + 1;
				}
				else {
					if (P[mid].y > P[P.size() - 1].y) {
						left = mid + 1;
					}
					else {
						right = mid;
					}
				}
			}
		}
		return left;
	}
	int highest(std::vector<Point>& P) {
		int left = 0;
		int right = P.size() - 1;
		if (P[0].y < P[P.size() - 1].y) {
			while (left < right) {
				int mid = (left + right) / 2;
				if (P[mid].y < P[mid + 1].y) {
					left = mid + 1;
				}
				else {
					if (P[mid].y < P[P.size() - 1].y) {
						left = mid + 1;
					}
					else {
						right = mid;
					}
				}
			}
		}
		else {
			while (left < right) {
				int mid = (left + right) / 2;
				if (P[mid].y > P[mid + 1].y) {
					right = mid;
				}
				else {
					if (P[mid].y < P[0].y) {
						right = mid - 1;
					}
					else {
						left = mid + 1;
					}
				}
			}
		}
		return left;
	}
	void monotonePartitioning(std::vector<Point>& P, std::vector<Point>& PL, std::vector<Point>& PR) {
		int low = lowest(P);
		int high = highest(P);
		if (low < high) {
			PL.insert(PL.end(), P.begin() + high, P.end());
			PL.insert(PL.end(), P.begin(), P.begin() + (low + 1));
			PR.insert(PR.end(), P.begin() + low, P.begin() + (high + 1));
		}
		else {
			PL.insert(PL.end(), P.begin() + high, P.begin() + (low + 1));
			PR.insert(PR.end(), P.begin() + low, P.end());
			PR.insert(PR.end(), P.begin(), P.begin() + (high + 1));
		}
	}
	bool semiinfiniteConvexChainIntersectionDetection(std::vector<Point>& L, std::vector<Point>& R) {
		int leftL = 0;
		int rightL = L.size() - 1;
		int leftR = 0;
		int rightR = R.size() - 1;
		while (true) {
			if (leftL < rightL && leftR < rightR) {
				int midL = (leftL + rightL) / 2;
				int midR = (leftR + rightR) / 2;
				Segment sL, sR;
				sL.first = L[midL];
				sL.second = L[midL + 1];
				sR.first = R[midR];
				sR.second = R[midR + 1];
				Point p = intersectionPoint(sL, sR);
				bool L1 = toLeft(sL.first, sL.second, sR.first);
				bool L2 = toLeft(sL.first, sL.second, sR.second);
				bool R1 = toLeft(sR.first, sR.second, sL.first);
				bool R2 = toLeft(sR.first, sR.second, sL.second);
				if (L1 != L2 && R1 != R2) {
					return true;
				}
				else if (!L1 && !L2 && !R1 && !R2) {
					if (sL.first.y < p.y) {
						rightL = midL;
						leftR = midR + 1;
					}
					else {
						leftL = midL + 1;
						rightR = midR;
					}
				}
				else if (L1 && L2 && R1 && R2) {
					if (sL.first.y < p.y) {
						if (sL.first.y < sR.second.y) {
							rightL = midL;
						}
						else {
							leftR = midR + 1;
						}
					}
					else {
						if (sL.second.y < sR.first.y) {
							rightR = midR;
						}
						else {
							leftL = midL + 1;
						}
					}
				}
				else if (L1 && L2 && !R1 && !R2) {
					if (sL.first.y < sR.first.y) {
						rightL = midL;
					}
					else {
						leftL = midL + 1;
					}
				}
				else if (!L1 && !L2 && R1 && R2) {
					if (sL.first.y < sR.first.y) {
						rightR = midR;
					}
					else {
						leftR = midR + 1;
					}
				}
				else if (L1 && !L2 && !R1 && !R2) {
					leftL = midL + 1;
					rightR = midR + 1;
				}
				else if (L1 && L2 && R1 && !R2) {
					rightL = midL + 1;
					rightR = midR;
				}
				else if (!L1 && !L2 && !R1 && R2) {
					leftL = midL;
					rightR = midR;
				}
				else if (!L1 && L2 && R1 && R2) {
					leftL = midL + 1;
					leftR = midR;
				}
				else if (L1 && L2 && !R1 && R2) {
					leftL = midL;
					leftR = midR + 1;
				}
				else if (!L1 && L2 && !R1 && !R2) {
					rightL = midL;
					leftR = midR;
				}
				else if (L1 && !L2 && R1 && R2) {
					rightL = midL;
					rightR = midR + 1;
				}
				else if (!L1 && !L2 && R1 && !R2) {
					rightL = midL + 1;
					leftR = midR + 1;
				}
			}
			else if (leftL < rightL && leftR == rightR) {
				int midL = (leftL + rightL) / 2;
				Segment sL;
				sL.first = L[midL];
				sL.second = L[midL + 1];
				Point p = R[leftR];
				if (p.y < sL.second.y) {
					leftL = midL + 1;
				}
				else if (p.y > sL.first.y) {
					rightL = midL;
				}
				else {
					if (toLeft(sL.first, sL.second, p)) {
						return true;
					}
					else {
						return false;
					}
				}
			}
			else if (leftL == rightL && leftR < rightR) {
				int midR = (leftR + rightR) / 2;
				Segment sR;
				sR.first = R[midR];
				sR.second = R[midR + 1];
				Point p = L[leftL];
				if (p.y > sR.second.y) {
					leftR = midR + 1;
				}
				else if (p.y < sR.first.y) {
					rightR = midR;
				}
				else {
					if (toLeft(sR.first, sR.second, p)) {
						return true;
					}
					else {
						return false;
					}
				}
			}
			else if (leftL == rightL && leftR == rightR) {
				return false;
			}
		}
	}
public:
	bool bruteForce(std::vector<Point>& P1, std::vector<Point>& P2) {
		for (int i = 0; i < P1.size(); i++) {
			Segment s1;
			s1.first = P1[i];
			s1.second = P1[i == P1.size() - 1 ? 0 : i + 1];
			for (int j = 0; j < P2.size(); j++) {
				Segment s2;
				s2.first = P2[j];
				s2.second = P2[j == P2.size() - 1 ? 0 : j + 1];
				if (intersect(s1, s2)) {
					return true;
				}
			}
		}
		return inConvexPolygon(P1, P2) || inConvexPolygon(P2, P1);
	}
	bool DobkinKirkpatrick(std::vector<Point>& P1, std::vector<Point>& P2) {
		std::vector<Point> P1L, P1R, P2L, P2R;
		monotonePartitioning(P1, P1L, P1R);
		monotonePartitioning(P2, P2L, P2R);
		return semiinfiniteConvexChainIntersectionDetection(P1L, P2R) && semiinfiniteConvexChainIntersectionDetection(P2L, P1R);
	}
};
