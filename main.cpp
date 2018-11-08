#include <vector>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <math.h>

#define M_PI 3.14159265358979323846
using namespace std;

struct state {
	double x;
	double y;
	double theta;
};

struct trajectory {
	double finalX;
	double finalY;
	double finalTheta;
	double time;
	double inputWheelLeft;
	double inputWheelRight;
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

vector<trajectory> generateTrajectory(state initial, state target) {
	vector<trajectory> answer;
	trajectory value;

	//Turn to parallel of X axis
	if (target.x >= initial.x) {
		value.finalX = initial.x;
		value.finalY = initial.y;
		value.finalTheta = 0;
		value.time = 0.2;
		value.inputWheelLeft = initial.theta/value.time;
		value.inputWheelRight = -value.inputWheelLeft;

		answer.push_back(value);
	}
	else {
		value.finalX = initial.x;
		value.finalY = initial.y;
		value.finalTheta = M_PI;
		value.time = 0.2;
		value.inputWheelLeft = initial.theta/value.time + M_PI/value.time;
		value.inputWheelRight = -value.inputWheelLeft;

		answer.push_back(value);
	}
	//Move under target
	value.finalX = target.x;
	value.finalY = initial.y;
	if (target.x >= initial.x) {
		value.finalTheta = 0;
	}
	else {
		value.finalTheta = M_PI;
	}
	value.time = 0.2;
	value.inputWheelLeft = (sqrt(pow(target.x - initial.x, 2)) / value.time) /20;
	value.inputWheelRight = (sqrt(pow(target.x - initial.x, 2)) / value.time) / 20;
	answer.push_back(value);

	//Turn to parallel of Y Axis
	if (target.y >= initial.y) {
		value.finalX = target.x;
		value.finalY = initial.y;
		value.finalTheta = M_PI / 2;
		value.time = 0.2;
		if (target.x >= initial.x) {
			value.inputWheelLeft = (3*M_PI/2)/ value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
		else {
			value.inputWheelLeft = (M_PI / 2) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
		

		answer.push_back(value);
	}
	else {
		value.finalX = target.x;
		value.finalY = initial.y;
		value.finalTheta = 3*M_PI/2;
		value.time = 0.2;
		if (target.x >= initial.x) {
			value.inputWheelLeft = (M_PI / 2) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
		else {
			value.inputWheelLeft = (3 * M_PI / 2) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}

		answer.push_back(value);
	}

	//Move to target
	value.finalX = target.x;
	value.finalY = target.y;
	if (target.y >= initial.y) {
		value.finalTheta = M_PI/2;
	}
	else {
		value.finalTheta = 3*M_PI/2;
	}
	value.time = 0.2;
	value.inputWheelLeft = (sqrt(pow(target.y - initial.y, 2)) / value.time) / 20;
	value.inputWheelRight = (sqrt(pow(target.y - initial.y, 2)) / value.time) / 20;
	answer.push_back(value);

	//Turn to have same target theta
	value.finalX = target.x;
	value.finalY = target.y;
	value.finalTheta = target.theta;
	value.time = 0.2;
	if (target.y >= initial.y) {
		if (target.theta >= M_PI / 2) {
			value.inputWheelLeft = (2 * M_PI - (target.theta - M_PI / 2)) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
		else {
			value.inputWheelLeft = (M_PI / 2 - target.theta) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
	}
	else {
		if (target.theta >= 3 * M_PI / 2) {
			value.inputWheelLeft = (2 * M_PI - (target.theta - 3 * M_PI / 2)) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
		else {
			value.inputWheelLeft = (3 * M_PI / 2 - target.theta) / value.time;
			value.inputWheelRight = -value.inputWheelLeft;
		}
	}

	answer.push_back(value);
	return answer;
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


int main(int argc, char** argv) {
	vector<state> V;
	state target;
	state initial;
	state temp;

	target.x = 1;
	target.y = 1;
	
	
	temp.x = 1;
	temp.y = 2;
	V.push_back(temp);

	temp.x = 4;
	temp.y = 1;
	V.push_back(temp);

	temp.x = 2;
	temp.y = 1;
	V.push_back(temp);

	temp.x = 1;
	temp.y = 3;
	V.push_back(temp);

	vector<state> resultNearest = nearestNeighbors(V, target, 3);
	for (int i = 0; i < resultNearest.size(); i++) {
		cout << resultNearest.at(i).x << " " << resultNearest.at(i).y << "\n";
	}

	target.x = 5;
	target.y = 5;
	target.theta = 0;

	initial.x = 0;
	initial.y = 0;
	initial.theta = 3 * M_PI / 2;
	
	vector<trajectory> resultTrajectory = generateTrajectory(initial, target);
	for (int i = 0; i < resultTrajectory.size(); i++) {
		cout << resultTrajectory.at(i).finalX << " " << resultTrajectory.at(i).finalY << " "
			<< resultTrajectory.at(i).inputWheelLeft << " " << resultTrajectory.at(i).inputWheelRight << " "
			<< resultTrajectory.at(i).time << "\n";
	}

}