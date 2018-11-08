#include <vector>
#include <iostream>
#include <algorithm>
#include <tuple>
using namespace std;

struct state {
	double x;
	double y;
	double theta;
};

struct trajectory {
	double x;
	double y;
	double theta;
	double time;
	double inputWheel1;
	double inputWheel2;
};

struct obstacle {
	double aX;
	double aY;
	double dX;
	double dY;
};

bool compareValues(tuple<state, double> i, tuple<state, double> j) {
	return get<1>(i) < get<1>(j);
}

vector<state> nearestNeighbors(vector<state> V, state target, int limit) {
	vector<state> answer;
	vector<tuple<state,double>> distanceValue;

	for (int i = 0; i < V.size(); i++) {
		distanceValue.push_back(make_tuple(V.at(i), sqrt(pow(target.x - V.at(i).x,2) + pow(target.y - V.at(i).y,2))));
	}

	for (int i = 0; i < distanceValue.size(); i++) {
		sort(distanceValue.begin(), distanceValue.end(), compareValues);
	}

	for (int i = 0; i < limit; i++) {
		answer.push_back(get<0>(distanceValue.at(i)));
	}
	return answer;
}

vector<trajctory> generateTrajectory(state initial, state target) {
	vector<trajectory> answer;

	if (target.x >= initial.x) {
		trajectory value;
		value.x = initial.x;
		value.y = initial.y;
		value.theta = 0;
		value.time = 0.2;
		value.inputWheel1 = (target.theta;
		value.inputWheel2 = 0;

		answer.push_back()
	}

}

bool collisionCheck(vector<obstacle> obstacles, state initial, state target) {
	for (int i = 0; i < obstacles.size(); i++) {
		double y1 = ((target.y - initial.y)*(obstacles.at(i).aX - initial.x)) / (target.x - initial.x) + initial.y;
		double y2 = ((target.y - initial.y)*(obstacles.at(i).dX - initial.x)) / (target.x - initial.x) + initial.y;
		double x1 = ((target.x - initial.x)*(obstacles.at(i).aY - initial.y)) / (target.y - initial.y) + initial.x;
		double x2 = ((target.x - initial.x)*(obstacles.at(i).aY - initial.y)) / (target.y - initial.y) + initial.x;

		if (x1 >= obstacles.at(i).aX && x1 <= obstacles.at(i).dX) {
			return true;
		}
		if (x2 >= obstacles.at(i).aX && x2 <= obstacles.at(i).dX) {
			return true;
		}
		if (y1 >= obstacles.at(i).aY && y1 <= obstacles.at(i).dY) {
			return true;
		}
		if (y2 >= obstacles.at(i).aY && y2 <= obstacles.at(i).dY) {
			return true;
		}
	}
	return false;
}


int main(int argc, char** argv[]) {

}