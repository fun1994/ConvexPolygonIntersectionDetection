#pragma once
#include <vector>
#include "Point.h"

class ConvexHull {
	double area2(Point& p, Point& q, Point& r) {
		return p.x * q.y - p.y * q.x + q.x * r.y - q.y * r.x + r.x * p.y - r.y * p.x;
	}
	bool toLeft(Point& p, Point& q, Point& r) {
		return area2(p, q, r) > 0;
	}
	bool toRight(Point& p, Point& q, Point& r) {
		return area2(p, q, r) < 0;
	}
	bool isCollinear(Point& p, Point& q, Point& r) {
		return area2(p, q, r) == 0;
	}
	bool between(Point& p, Point& q, Point& r) {
		return (q.x - p.x) * (r.x - q.x) + (q.y - p.y) * (r.y - q.y) > 0;
	}
	bool inTriangle(Point& p, Point& q, Point& r, Point& s) {
		if (toLeft(p, q, r)) {
			return toLeft(p, q, s) && toLeft(q, r, s) && toLeft(r, p, s);
		}
		else if (toRight(p, q, r)) {
			return toRight(p, q, s) && toRight(q, r, s) && toRight(r, p, s);
		}
	}
	void checkEdge(std::vector<Point>& S, int p, int q) {
		bool LEmpty = true;
		bool REmpty = true;
		for (int k = 0; k < S.size() && (LEmpty || REmpty); k++) {
			if (k != p && k != q) {
				if (toLeft(S[p], S[q], S[k])) {
					LEmpty = false;
				}
				else if (toRight(S[p], S[q], S[k])) {
					REmpty = false;
				}
				else {
					if (!between(S[p], S[k], S[q])) {
						LEmpty = false;
						REmpty = false;
					}
				}
			}
		}
		if (LEmpty || REmpty) {
			S[p].extreme = true;
			S[q].extreme = true;
		}
	}
	bool inConvexPolygon(std::vector<Point>& S, Point& p) {
		for (int i = 0; i < S.size(); i++) {
			if (!toLeft(S[i], S[i == S.size() - 1 ? 0 : i + 1], p)) {
				return false;
			}
		}
		return true;
	}
	char patternOfTurn(std::vector<Point>& S, Point& x, int i) {
		if ((toLeft(x, S[i], S[i == 0 ? S.size() - 1 : i - 1]) || (isCollinear(x, S[i], S[i == 0 ? S.size() - 1 : i - 1]) && between(x, S[i == 0 ? S.size() - 1 : i - 1], S[i]))) && toLeft(x, S[i], S[i == S.size() - 1 ? 0 : i + 1])) {
			return 's';
		}
		else if (toRight(x, S[i], S[i == 0 ? S.size() - 1 : i - 1]) && (toRight(x, S[i], S[i == S.size() - 1 ? 0 : i + 1]) || (isCollinear(x, S[i], S[i == S.size() - 1 ? 0 : i + 1]) && between(x, S[i == S.size() - 1 ? 0 : i + 1], S[i])))) {
			return 't';
		}
		else {
			return 'v';
		}
	}
	void tangentPoint(std::vector<Point>& S, Point& x, int& s, int& t) {
		for (int i = 0; i < S.size(); i++) {
			char pattern = patternOfTurn(S, x, i);
			if (pattern == 's') {
				s = i;
			}
			else if (pattern == 't') {
				t = i;
			}
		}
	}
	int LTL(std::vector<Point>& S) {
		int ltl = 0;
		for (int k = 1; k < S.size(); k++) {
			if (S[k].y < S[ltl].y || (S[k].y == S[ltl].y && S[k].x < S[ltl].x)) {
				ltl = k;
			}
		}
		return ltl;
	}
	int tangentEdge(std::vector<Point>& S, int k) {
		int s = -1;
		for (int t = 0; t < S.size(); t++) {
			if (t != k && (s == -1 || toRight(S[k], S[s], S[t]) || (isCollinear(S[k], S[s], S[t]) && between(S[k], S[s], S[t])))) {
				s = t;
			}
		}
		return s;
	}
	int polarAnglePartition(std::vector<Point>& S, int low, int high) {
		Point pivot = S[low];
		while (low < high) {
			while (low < high && (toLeft(S[0], pivot, S[high]) || (isCollinear(S[0], pivot, S[high]) && between(S[0], pivot, S[high])) || S[high] == pivot)) {
				high--;
			}
			S[low] = S[high];
			while (low < high && (toRight(S[0], pivot, S[low]) || (isCollinear(S[0], pivot, S[low]) && between(S[0], S[low], pivot)) || S[low] == pivot)) {
				low++;
			}
			S[high] = S[low];
		}
		S[low] = pivot;
		return low;
	}
	void polarAngleQuickSort(std::vector<Point>& S, int low, int high) {
		if (low < high) {
			int pivot = polarAnglePartition(S, low, high);
			polarAngleQuickSort(S, low, pivot - 1);
			polarAngleQuickSort(S, pivot + 1, high);
		}
	}
	void polarAngleQuickSort(std::vector<Point>& S) {
		polarAngleQuickSort(S, 1, S.size() - 1);
	}
	void scan(std::vector<Point>& S, std::vector<Point>& T) {
		while (!T.empty()) {
			if (S.size() < 2 || toLeft(S[S.size() - 2], S[S.size() - 1], T[T.size() - 1])) {
				S.push_back(T[T.size() - 1]);
				T.pop_back();
			}
			else {
				S.pop_back();
			}
		}
	}
	std::vector<Point> merge(std::vector<Point>& S1, std::vector<Point>& S2) {
		std::vector<Point> S;
		int ltl1 = LTL(S1);
		int ltl2 = LTL(S2);
		if (S2[ltl2].y < S1[ltl1].y || (S2[ltl2].y == S1[ltl1].y && S2[ltl2].x < S1[ltl1].x)) {
			ltl1 = ltl2;
			std::swap(S1, S2);
		}
		S1.insert(S1.end(), S1.begin(), S1.begin() + ltl1);
		S1.erase(S1.begin(), S1.begin() + ltl1);
		if (S2.size() == 2) {
			if (toLeft(S1[0], S2[1], S2[0]) || (isCollinear(S1[0], S2[1], S2[0]) && between(S1[0], S2[1], S2[0]))) {
				std::swap(S2[0], S2[1]);
			}
		}
		else if (S2.size() > 2) {
			int s = -1;
			int t = -1;
			tangentPoint(S2, S1[0], s, t);
			if (s != -1 && t != -1) {
				if (s < t) {
					S2.erase(S2.begin() + (t + 1), S2.end());
					S2.erase(S2.begin(), S2.begin() + s);
				}
				else if (s > t) {
					S2.insert(S2.end(), S2.begin(), S2.begin() + (t + 1));
					S2.erase(S2.begin(), S2.begin() + s);
				}
			}
		}
		S.push_back(S1[0]);
		int index1 = 1;
		int index2 = 0;
		while (index1 < S1.size() && index2 < S2.size()) {
			if (toLeft(S[0], S1[index1], S2[index2]) || (isCollinear(S[0], S1[index1], S2[index2]) && between(S[0], S1[index1], S2[index2]))) {
				S.push_back(S1[index1]);
				index1++;
			}
			else {
				S.push_back(S2[index2]);
				index2++;
			}
		}
		if (index1 == S1.size()) {
			S.insert(S.end(), S2.begin() + index2, S2.end());
		}
		else {
			S.insert(S.end(), S1.begin() + index1, S1.end());
		}
		return S;
	}
	std::vector<Point> divideAndConquer1(std::vector<Point>& S, int low, int high) {
		std::vector<Point> convexHull;
		if (high - low < 2) {
			convexHull.insert(convexHull.end(), S.begin() + low, S.begin() + high);
		}
		else {
			int mid = (low + high) / 2;
			std::vector<Point> convexHull1 = divideAndConquer1(S, low, mid);
			std::vector<Point> convexHull2 = divideAndConquer1(S, mid, high);
			std::vector<Point> T = merge(convexHull1, convexHull2);
			convexHull.insert(convexHull.begin(), T.begin(), T.begin() + 2);
			T.erase(T.begin(), T.begin() + 2);
			reverse(T.begin(), T.end());
			scan(convexHull, T);
		}
		return convexHull;
	}
	int yPartition(std::vector<Point>& S, int low, int high) {
		Point pivot = S[low];
		while (low < high) {
			while (low < high && (S[high].y > pivot.y || (S[high].y == pivot.y && S[high].x >= pivot.x))) {
				high--;
			}
			S[low] = S[high];
			while (low < high && (S[low].y < pivot.y || (S[low].y == pivot.y && S[low].x <= pivot.x))) {
				low++;
			}
			S[high] = S[low];
		}
		S[low] = pivot;
		return low;
	}
	void yQuickSort(std::vector<Point>& S, int low, int high) {
		if (low < high) {
			int pivot = yPartition(S, low, high);
			yQuickSort(S, low, pivot - 1);
			yQuickSort(S, pivot + 1, high);
		}
	}
	void yQuickSort(std::vector<Point>& S) {
		yQuickSort(S, 0, S.size() - 1);
	}
	void commonTangent(std::vector<Point>& S1, std::vector<Point>& S2, int htr1, int& s1, int& t1, int& s2, int& t2) {
		s1 = htr1;
		t2 = 0;
		bool flag = true;
		while (true) {
			if (flag) {
				if (patternOfTurn(S2, S1[s1], t2) == 't') {
					if (patternOfTurn(S1, S2[t2], s1) == 's') {
						break;
					}
					else {
						s1 == S1.size() - 1 ? s1 = 0 : s1++;
						flag = false;
					}
				}
				else {
					t2 == 0 ? t2 = S2.size() - 1 : t2--;
				}
			}
			else {
				if (patternOfTurn(S1, S2[t2], s1) == 's') {
					if (patternOfTurn(S2, S1[s1], t2) == 't') {
						break;
					}
					else {
						t2 == 0 ? t2 = S2.size() - 1 : t2--;
						flag = true;
					}
				}
				else {
					s1 == S1.size() - 1 ? s1 = 0 : s1++;
				}
			}
		}
		t1 = htr1;
		s2 = 0;
		flag = true;
		while (true) {
			if (flag) {
				if (patternOfTurn(S2, S1[t1], s2) == 's') {
					if (patternOfTurn(S1, S2[s2], t1) == 't') {
						break;
					}
					else {
						t1 == 0 ? t1 = S1.size() - 1 : t1--;
						flag = false;
					}
				}
				else {
					s2 == S2.size() - 1 ? s2 = 0 : s2++;
				}
			}
			else {
				if (patternOfTurn(S1, S2[s2], t1) == 't') {
					if (patternOfTurn(S2, S1[t1], s2) == 's') {
						break;
					}
					else {
						s2 == S2.size() - 1 ? s2 = 0 : s2++;
						flag = true;
					}
				}
				else {
					t1 == 0 ? t1 = S1.size() - 1 : t1--;
				}
			}
		}
	}
	std::vector<Point> stitch(std::vector<Point>& S1, std::vector<Point>& S2, int htr1, int htr2, int& htr) {
		std::vector<Point> S;
		if (S1.size() == 1 && S2.size() == 1) {
			S.push_back(S1[0]);
			S.push_back(S2[0]);
			htr = 1;
		}
		else if (S1.size() == 1 && S2.size() == 2) {
			S.push_back(S1[0]);
			if (toLeft(S1[0], S2[0], S2[1])) {
				S.push_back(S2[0]);
				S.push_back(S2[1]);
				htr = 2;
			}
			else if (toRight(S1[0], S2[0], S2[1])) {
				S.push_back(S2[1]);
				S.push_back(S2[0]);
				htr = 1;
			}
			else {
				S.push_back(S2[1]);
				htr = 1;
			}
		}
		else if (S1.size() == 2 && S2.size() == 2) {
			int s1, t1, s2, t2;
			if (toRight(S1[1], S2[0], S2[1])) {
				if (toLeft(S2[0], S1[1], S1[0])) {
					s1 = 1;
					t2 = 0;
				}
				else {
					if (toRight(S1[0], S2[0], S2[1])) {
						s1 = 0;
						t2 = 0;
					}
					else {
						s1 = 0;
						t2 = 1;
					}
				}
			}
			else {
				if (toLeft(S2[1], S1[1], S1[0])) {
					s1 = 1;
					t2 = 1;
				}
				else {
					s1 = 0;
					t2 = 1;
				}
			}
			if (toLeft(S1[1], S2[0], S2[1])) {
				if (toRight(S2[0], S1[1], S1[0])) {
					t1 = 1;
					s2 = 0;
				}
				else {
					if (toLeft(S1[0], S2[0], S2[1])) {
						t1 = 0;
						s2 = 0;
					}
					else {
						t1 = 0;
						s2 = 1;
					}
				}
			}
			else {
				if (toRight(S2[1], S1[1], S1[0])) {
					t1 = 1;
					s2 = 1;
				}
				else {
					t1 = 0;
					s2 = 1;
				}
			}
			S.push_back(S1[0]);
			if (s1 == 0 && t1 == 0 && s2 == 0 && t2 == 1) {
				S.push_back(S2[0]);
				S.push_back(S2[1]);
				htr = 2;
			}
			else if (s1 == 0 && t1 == 0 && s2 == 1 && t2 == 0) {
				S.push_back(S2[1]);
				S.push_back(S2[0]);
				htr = 1;
			}
			else if (s1 == 0 && t1 == 0 && s2 == 1 && t2 == 1) {
				S.push_back(S2[1]);
				htr = 1;
			}
			else if (s1 == 0 && t1 == 1 && s2 == 0 && t2 == 1) {
				S.push_back(S1[1]);
				S.push_back(S2[0]);
				S.push_back(S2[1]);
				htr = 3;
			}
			else if (s1 == 0 && t1 == 1 && s2 == 1 && t2 == 0) {
				S.push_back(S1[1]);
				S.push_back(S2[1]);
				S.push_back(S2[0]);
				htr = 2;
			}
			else if (s1 == 0 && t1 == 1 && s2 == 1 && t2 == 1) {
				S.push_back(S1[1]);
				S.push_back(S2[1]);
				htr = 2;
			}
			else if (s1 == 1 && t1 == 0 && s2 == 0 && t2 == 1) {
				S.push_back(S2[0]);
				S.push_back(S2[1]);
				S.push_back(S1[1]);
				htr = 2;
			}
			else if (s1 == 1 && t1 == 0 && s2 == 1 && t2 == 0) {
				S.push_back(S2[1]);
				S.push_back(S2[0]);
				S.push_back(S1[1]);
				htr = 1;
			}
			else if (s1 == 1 && t1 == 0 && s2 == 1 && t2 == 1) {
				S.push_back(S2[1]);
				S.push_back(S1[1]);
				htr = 1;
			}
		}
		else if (S1.size() == 2 && S2.size() > 2) {
			int s1;
			int t2 = 0;
			while (patternOfTurn(S2, S1[1], t2) != 't') {
				t2 == 0 ? t2 = S2.size() - 1 : t2--;
			}
			if (toLeft(S2[t2], S1[1], S1[0])) {
				s1 = 1;
			}
			else {
				s1 = 0;
				while (patternOfTurn(S2, S1[0], t2) != 't') {
					t2 == 0 ? t2 = S2.size() - 1 : t2--;
				}
			}
			int t1;
			int s2 = 0;
			while (patternOfTurn(S2, S1[1], s2) != 's') {
				s2 == S2.size() - 1 ? s2 = 0 : s2++;
			}
			if (toRight(S2[s2], S1[1], S1[0])) {
				t1 = 1;
			}
			else {
				t1 = 0;
				while (patternOfTurn(S2, S1[0], s2) != 's') {
					s2 == S2.size() - 1 ? s2 = 0 : s2++;
				}
			}
			S.push_back(S1[0]);
			if (s1 == 0 && t1 == 0) {
				S.insert(S.end(), S2.begin() + s2, t2 == 0 ? S2.end() : S2.begin() + (t2 + 1));
				if (t2 == 0) {
					S.push_back(S2[0]);
				}
				htr = htr2 - s2 + 1;
			}
			else if (s1 == 0 && t1 == 1) {
				S.push_back(S1[1]);
				S.insert(S.end(), S2.begin() + s2, t2 == 0 ? S2.end() : S2.begin() + (t2 + 1));
				if (t2 == 0) {
					S.push_back(S2[0]);
				}
				htr = htr2 - s2 + 2;
			}
			else if (s1 == 1 && t1 == 0) {
				S.insert(S.end(), S2.begin() + s2, t2 == 0 ? S2.end() : S2.begin() + (t2 + 1));
				if (t2 == 0) {
					S.push_back(S2[0]);
				}
				S.push_back(S1[1]);
				htr = htr2 - s2 + 1;
			}
		}
		else if (S1.size() > 2 && S2.size() == 2) {
			int s1 = htr1;
			int t2;
			while (patternOfTurn(S1, S2[0], s1) != 's') {
				s1 == S1.size() - 1 ? s1 = 0 : s1++;
			}
			if (toRight(S1[s1], S2[0], S2[1])) {
				t2 = 0;
			}
			else {
				t2 = 1;
				while (patternOfTurn(S1, S2[1], s1) != 's') {
					s1 == S1.size() - 1 ? s1 = 0 : s1++;
				}
			}
			int t1 = htr1;
			int s2;
			while (patternOfTurn(S1, S2[0], t1) != 't') {
				t1 == 0 ? t1 = S1.size() - 1 : t1--;
			}
			if (toLeft(S1[t1], S2[0], S2[1])) {
				s2 = 0;
			}
			else {
				s2 = 1;
				while (patternOfTurn(S1, S2[1], t1) != 't') {
					t1 == 0 ? t1 = S1.size() - 1 : t1--;
				}
			}
			S.insert(S.end(), S1.begin(), S1.begin() + (t1 + 1));
			if (s2 == 0 && t2 == 1) {
				S.push_back(S2[0]);
				S.push_back(S2[1]);
				htr = t1 + 2;
			}
			else if (s2 == 1 && t2 == 0) {
				S.push_back(S2[1]);
				S.push_back(S2[0]);
				htr = t1 + 1;
			}
			else if (s2 == 1 && t2 == 1) {
				S.push_back(S2[1]);
				htr = t1 + 1;
			}
			if (s1 > 0) {
				S.insert(S.end(), S1.begin() + s1, S1.end());
			}
		}
		else {
			int s1, t1, s2, t2;
			commonTangent(S1, S2, htr1, s1, t1, s2, t2);
			S.insert(S.end(), S1.begin(), S1.begin() + (t1 + 1));
			S.insert(S.end(), S2.begin() + s2, t2 == 0 ? S2.end() : S2.begin() + (t2 + 1));
			if (t2 == 0) {
				S.push_back(S2[0]);
			}
			if (s1 > 0) {
				S.insert(S.end(), S1.begin() + s1, S1.end());
			}
			htr = htr2 - s2 + t1 + 1;
		}
		return S;
	}
	std::vector<Point> divideAndConquer2(std::vector<Point>& S, int low, int high, int& htr) {
		std::vector<Point> convexHull;
		if (high - low < 2) {
			convexHull.insert(convexHull.end(), S.begin() + low, S.begin() + high);
			htr = 0;
		}
		else {
			int mid = (low + high) / 2;
			int htr1;
			int htr2;
			std::vector<Point> convexHull1 = divideAndConquer2(S, low, mid, htr1);
			std::vector<Point> convexHull2 = divideAndConquer2(S, mid, high, htr2);
			convexHull = stitch(convexHull1, convexHull2, htr1, htr2, htr);
		}
		return convexHull;
	}
public:
	std::vector<Point> extremePoint(std::vector<Point>& S) {
		for (int s = 0; s < S.size(); s++) {
			S[s].extreme = true;
		}
		for (int p = 0; p < S.size(); p++) {
			for (int q = p + 1; q < S.size(); q++) {
				for (int r = q + 1; r < S.size(); r++) {
					if (isCollinear(S[p], S[q], S[r])) {
						if (between(S[p], S[q], S[r])) {
							S[q].extreme = false;
						}
						else if (between(S[q], S[r], S[p])) {
							S[r].extreme = false;
						}
						else {
							S[p].extreme = false;
						}
					}
					else {
						for (int s = 0; s < S.size(); s++) {
							if (s != p && s != q && s != r && S[s].extreme) {
								if (inTriangle(S[p], S[q], S[r], S[s])) {
									S[s].extreme = false;
								}
							}
						}
					}
				}
			}
		}
		std::vector<Point> convexHull;
		for (int s = 0; s < S.size(); s++) {
			if (S[s].extreme) {
				convexHull.push_back(S[s]);
			}
		}
		if (!convexHull.empty()) {
			int ltl = LTL(convexHull);
			std::swap(convexHull[0], convexHull[ltl]);
			polarAngleQuickSort(convexHull);
		}
		return convexHull;
	}
	std::vector<Point> extremeEdge(std::vector<Point>& S) {
		if (S.size() == 1) {
			S[0].extreme = true;
		}
		else {
			for (int s = 0; s < S.size(); s++) {
				S[s].extreme = false;
			}
			for (int p = 0; p < S.size(); p++) {
				for (int q = p + 1; q < S.size(); q++) {
					checkEdge(S, p, q);
				}
			}
		}
		std::vector<Point> convexHull;
		for (int s = 0; s < S.size(); s++) {
			if (S[s].extreme) {
				convexHull.push_back(S[s]);
			}
		}
		if (!convexHull.empty()) {
			int ltl = LTL(convexHull);
			std::swap(convexHull[0], convexHull[ltl]);
			polarAngleQuickSort(convexHull);
		}
		return convexHull;
	}
	std::vector<Point> incrementalConstruction(std::vector<Point>& S) {
		std::vector<Point> convexHull;
		for (int i = 0; i < S.size(); i++) {
			if (convexHull.size() < 2) {
				convexHull.push_back(S[i]);
			}
			else if (convexHull.size() == 2) {
				if (toLeft(convexHull[0], convexHull[1], S[i])) {
					convexHull.push_back(S[i]);
				}
				else if (toRight(convexHull[0], convexHull[1], S[i])) {
					convexHull.insert(convexHull.begin() + 1, S[i]);
				}
				else {
					if (between(convexHull[1], convexHull[0], S[i])) {
						convexHull[0] = S[i];
					}
					else if (between(convexHull[0], convexHull[1], S[i])) {
						convexHull[1] = S[i];
					}
				}
			}
			else {
				int s = -1;
				int t = -1;
				tangentPoint(convexHull, S[i], s, t);
				if (s != -1 && t != -1) {
					if (s < t) {
						convexHull.erase(convexHull.begin() + (t + 1), convexHull.end());
						convexHull.erase(convexHull.begin(), convexHull.begin() + s);
						convexHull.push_back(S[i]);
					}
					else if (s > t) {
						convexHull.erase(convexHull.begin() + (t + 1), convexHull.begin() + s);
						convexHull.insert(convexHull.begin() + (t + 1), S[i]);
					}
				}
			}
		}
		if (!convexHull.empty()) {
			int ltl = LTL(convexHull);
			convexHull.insert(convexHull.end(), convexHull.begin(), convexHull.begin() + ltl);
			convexHull.erase(convexHull.begin(), convexHull.begin() + ltl);
		}
		for (int i = 0; i < S.size(); i++) {
			S[i].extreme = false;
			for (int j = 0; j < convexHull.size(); j++) {
				if (S[i] == convexHull[j]) {
					S[i].extreme = true;
					break;
				}
			}
		}
		return convexHull;
	}
	std::vector<Point> JarvisMarch(std::vector<Point>& S) {
		std::vector<Point> convexHull;
		for (int k = 0; k < S.size(); k++) {
			S[k].extreme = false;
		}
		if (!S.empty()) {
			int ltl = LTL(S);
			int k = ltl;
			do {
				S[k].extreme = true;
				convexHull.push_back(S[k]);
				int s = tangentEdge(S, k);
				if (s != -1) {
					k = s;
				}
			} while (ltl != k);
		}
		return convexHull;
	}
	std::vector<Point> GrahamScan(std::vector<Point>& S) {
		std::vector<Point> convexHull;
		if (S.size() < 2) {
			for (int i = 0; i < S.size(); i++) {
				convexHull.push_back(S[i]);
			}
		}
		else {
			std::vector<Point> T = S;
			int ltl = LTL(T);
			std::swap(T[0], T[ltl]);
			polarAngleQuickSort(T);
			convexHull.insert(convexHull.begin(), T.begin(), T.begin() + 2);
			T.erase(T.begin(), T.begin() + 2);
			reverse(T.begin(), T.end());
			scan(convexHull, T);
		}
		for (int i = 0; i < S.size(); i++) {
			S[i].extreme = false;
			for (int j = 0; j < convexHull.size(); j++) {
				if (S[i] == convexHull[j]) {
					S[i].extreme = true;
					break;
				}
			}
		}
		return convexHull;
	}
	std::vector<Point> divideAndConquer1(std::vector<Point>& S) {
		std::vector<Point> convexHull = divideAndConquer1(S, 0, S.size());
		for (int i = 0; i < S.size(); i++) {
			S[i].extreme = false;
			for (int j = 0; j < convexHull.size(); j++) {
				if (S[i] == convexHull[j]) {
					S[i].extreme = true;
					break;
				}
			}
		}
		return convexHull;
	}
	std::vector<Point> divideAndConquer2(std::vector<Point>& S) {
		std::vector<Point> T = S;
		yQuickSort(T);
		int htr;
		std::vector<Point> convexHull = divideAndConquer2(T, 0, T.size(), htr);
		for (int i = 0; i < S.size(); i++) {
			S[i].extreme = false;
			for (int j = 0; j < convexHull.size(); j++) {
				if (S[i] == convexHull[j]) {
					S[i].extreme = true;
					break;
				}
			}
		}
		return convexHull;
	}
};
