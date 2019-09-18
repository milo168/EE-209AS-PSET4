#include <vector>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <math.h>
#include <random>
#include <thread>
#include <ctime>
#include <chrono>
#include <cmath>
#include <stack>

#define M_PI 3.14159265358979323846
using namespace std;
using namespace chrono;

state generateRandomState(obstacle mapBoundary, vector<state> chosen) {

	random_device rd; 
	mt19937 gen1(rd());
	mt19937 gen2(rd());
	mt19937 gen3(rd());
	state randState;

	uniform_real_distribution<double> xRand(mapBoundary.aX, mapBoundary.dX);
	uniform_real_distribution<double> yRand(mapBoundary.aY, mapBoundary.dY);
	uniform_real_distribution<double> thetaRand(0, 2*M_PI);

	randState.x =  xRand(gen1);
	randState.y = yRand(gen2);
	randState.theta = thetaRand(gen3);

	std::cout << "RANDOM STATE: " << randState.x << " " << randState.y << " " << randState.theta << "\n";

	for (int i = 0; i < chosen.size(); i++) {
		if (abs(randState.x - chosen.at(i).x) < 0.00001 && abs(randState.y - chosen.at(i).y) < 0.00001 && abs(randState.theta - chosen.at(i).theta) < 0.00001) {
			randState.x = xRand(gen1);
			randState.y = yRand(gen2);
			randState.theta = thetaRand(gen3);
			i = -1;
		}
	}
	return randState;
}

int main(int argc, char** argv) {
}