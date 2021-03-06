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

struct state {
	double x;
	double y;
	double theta;
};

struct trajectory {
	state finalState;
	double time;
	double inputWheelLeft;
	double inputWheelRight;
};

struct obstacle {
	double aX;
	double aY;
	double dX;
	double dY;
	int type;
};

bool compareValues(tuple<state, double> i, tuple<state, double> j) {
	return get<1>(i) < get<1>(j);
}

//Find nearest node in the graph
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

//Generate trajectory from initial state to final state
vector<trajectory> generateTrajectory(state initial, state target) {
	vector<trajectory> answer;
	trajectory value;
	value.finalState = initial;
	value.time = 0;
	value.inputWheelLeft = 0;
	value.inputWheelRight = 0;
	answer.push_back(value);

	//Turn to parallel of X axis
	if (target.x >= initial.x) {
		value.finalState.x = initial.x;
		value.finalState.y = initial.y;
		value.finalState.theta = 0;
		value.time = 0.2;
		value.inputWheelLeft = initial.theta/value.time;
		value.inputWheelRight = -value.inputWheelLeft;

		answer.push_back(value);
	}
	else {
		value.finalState.x = initial.x;
		value.finalState.y = initial.y;
		value.finalState.theta = M_PI;
		value.time = 0.2;
		value.inputWheelLeft = initial.theta/value.time + M_PI/value.time;
		value.inputWheelRight = -value.inputWheelLeft;

		answer.push_back(value);
	}
	//Move under target
	value.finalState.x = target.x;
	value.finalState.y = initial.y;
	if (target.x >= initial.x) {
		value.finalState.theta = 0;
	}
	else {
		value.finalState.theta = M_PI;
	}
	value.time = 0.2;
	value.inputWheelLeft = (sqrt(pow(target.x - initial.x, 2)) / value.time) /20;
	value.inputWheelRight = (sqrt(pow(target.x - initial.x, 2)) / value.time) / 20;
	answer.push_back(value);

	//Turn to parallel of Y Axis
	if (target.y >= initial.y) {
		value.finalState.x = target.x;
		value.finalState.y = initial.y;
		value.finalState.theta = M_PI / 2;
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
		value.finalState.x = target.x;
		value.finalState.y = initial.y;
		value.finalState.theta = 3*M_PI/2;
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
	value.finalState.x = target.x;
	value.finalState.y = target.y;
	if (target.y >= initial.y) {
		value.finalState.theta = M_PI/2;
	}
	else {
		value.finalState.theta = 3*M_PI/2;
	}
	value.time = 0.2;
	value.inputWheelLeft = (sqrt(pow(target.y - initial.y, 2)) / value.time) / 20;
	value.inputWheelRight = (sqrt(pow(target.y - initial.y, 2)) / value.time) / 20;
	answer.push_back(value);

	//Turn to have same target theta
	value.finalState.x = target.x;
	value.finalState.y = target.y;
	value.finalState.theta = target.theta;
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

//Helper function since our program has a specific way of doing collision checking
void normalizeCoordinate(state initial, state target, state &newInitial, state &newTarget) {
	if (initial.x <= target.x && initial.y <= target.y) {
		newInitial.x = initial.x;
		newInitial.y = initial.y;
		newInitial.theta = initial.theta;

		newTarget.x = target.x;
		newTarget.y = target.y;
		newTarget.theta = target.theta;

		return;
	}
	if (initial.x <= target.x && initial.y >= target.y) {
		newInitial.x = initial.x;
		newInitial.y = target.y;
		newInitial.theta = initial.theta;

		newTarget.x = target.x;
		newTarget.y = initial.y;
		newTarget.theta = target.theta;

		return;
	}
	if (initial.x >= target.x && initial.y >= target.y) {
		newInitial.x = target.x;
		newInitial.y = target.y;
		newInitial.theta = initial.theta;

		newTarget.x = initial.x;
		newTarget.y = initial.y;
		newTarget.theta = target.theta;

		return;
	}
	if (initial.x >= target.x && initial.y <= target.y) {
		newInitial.x = target.x;
		newInitial.y = initial.y;
		newInitial.theta = initial.theta;

		newTarget.x = initial.x;
		newTarget.y = target.y;
		newTarget.theta = target.theta;

		return;
	}
}

bool collisionCheck(vector<obstacle> obstacles, state initial, state target) {
	for (int i = 0; i < obstacles.size(); i++) {
		/*double y1 = ((target.y - initial.y)*(obstacles.at(i).aX - initial.x)) / (target.x - initial.x) + initial.y;
		double y2 = ((target.y - initial.y)*(obstacles.at(i).dX - initial.x)) / (target.x - initial.x) + initial.y;
		double x1 = ((target.x - initial.x)*(obstacles.at(i).aY - initial.y)) / (target.y - initial.y) + initial.x;
		double x2 = ((target.x - initial.x)*(obstacles.at(i).aY - initial.y)) / (target.y - initial.y) + initial.x;

		if (obstacles.at(i).type == 0) {
			if (x1 > obstacles.at(i).aX && x1 < obstacles.at(i).dX) {
				cout << " FAILED HERE 5" << "\n";
				return true;
			}
			if (x2 > obstacles.at(i).aX && x2 < obstacles.at(i).dX) {
				cout << " FAILED HERE 6" << "\n";
				return true;
			}
			if (y1 > obstacles.at(i).aY && y1 < obstacles.at(i).dY) {
				cout << y1 << " " << obstacles.at(i).aY << " " << obstacles.at(i).dY << "\n";
				cout << " FAILED HERE 7" << "\n";
				return true;
			}
			if (y2 > obstacles.at(i).aY && y2 < obstacles.at(i).dY) {
				cout << " FAILED HERE 8" << "\n";
				return true;
			}
		}
		else if(obstacles.at(i).type == 1) {
			if (target.x > obstacles.at(i).dX || target.x < obstacles.at(i).aX) {
				cout << " FAILED HERE 1" << "\n";
				return true;
			}
			if (target.y > obstacles.at(i).dY || target.y < obstacles.at(i).aY) {
				cout << " FAILED HERE 2" << "\n";
				return true;
			}
		}*/
		if (obstacles.at(i).type == 0) {
			//left vertical
			double a1 = obstacles.at(i).dY - obstacles.at(i).aY;
			double b1 = 0;
			double c1 = a1*(obstacles.at(i).aX) + b1*(obstacles.at(i).aY);

			//right vertical
			double a2 = obstacles.at(i).dY - obstacles.at(i).aY;
			double b2 = 0;
			double c2 = a2*(obstacles.at(i).dX) + b2*(obstacles.at(i).dY);

			//top horizontal
			double a3 = 0;
			double b3 = obstacles.at(i).dX - obstacles.at(i).aX;
			double c3 = a3*(obstacles.at(i).dX) + b3*(obstacles.at(i).dY);

			//bottom horizontal
			double a4 = 0;
			double b4 = obstacles.at(i).dX - obstacles.at(i).aX;
			double c4 = a4*(obstacles.at(i).aX) + b4*(obstacles.at(i).aY);

			//target line
			double a5 = target.y - initial.y;
			double b5 = target.x - initial.x;
			double c5 = a5*(initial.x) + b5*(initial.y);

			double determinant1 = a1*b5 - a5*b1;
			double determinant2 = a2*b5 - a5*b2;
			double determinant3 = a3*b5 - a5*b3;
			double determinant4 = a4*b5 - a5*b4;

			double x1 = -1, x2 = -1, x3 = -1, x4 = -1;
			double y1 = -1, y2 = -1, y3 = -1, y4 = -1;
			if (determinant1 != 0)
			{
				x1 = (b5*c1 - b1*c5) / determinant1;
				y1 = (a1*c5 - a5*c1) / determinant1;
			}
			if (determinant2 != 0)
			{
				x2 = (b5*c2 - b2*c5) / determinant2;
				y2 = (a2*c5 - a5*c2) / determinant2;
			}
			if (determinant3 != 0)
			{
				x3 = (b5*c3 - b3*c5) / determinant3;
				y3 = (a3*c5 - a5*c3) / determinant3;
			}
			if (determinant4 != 0)
			{
				x4 = (b5*c4 - b4*c5) / determinant4;
				y4 = (a4*c5 - a5*c4) / determinant4;
			}

			state newInitial;
			state newTarget;
			normalizeCoordinate(initial, target, newInitial, newTarget);
			double eps = 1e5;
			
			/*cout << "INITIAL STUFF: " << initial.x << " " << initial.y << " " << target.x << " " << target.y << "\n";
			cout << "NORMALIZED: " << newInitial.x << " " << newInitial.y << " " << newTarget.x << " " << newTarget.y << "\n";
			cout << "OBSTACLES: " << obstacles.at(i).aX << " " << obstacles.at(i).aY << " " << obstacles.at(i).dX << " " << obstacles.at(i).dY << "\n";
			cout << x1 << " " << y1 << " CHECK1" << "\n";
			cout << x2 << " " << y2 << " CHECK2" << "\n";
			cout << x3 << " " << y3 << " CHECK3" << "\n";
			cout << x4 << " " << y4 << " CHECK4" << "\n\n";*/

			//if the intersection point lies within these boundaries it collides
			if ((int)(x1*eps) >= (int)(obstacles.at(i).aX*eps) && (int)(x1*eps) <= (int)(obstacles.at(i).dX*eps) && (int)(y1*eps) >= (int)(obstacles.at(i).aY*eps) && (int)(y1*eps) <= (int)(obstacles.at(i).dY*eps) &&
				(int)(x1*eps) >= (int)(newInitial.x*eps) && (int)(x1*eps) <= (int)(newTarget.x*eps) && (int)(y1*eps) >= (int)(newInitial.y*eps) && (int)(y1*eps) <= (int)(newTarget.y*eps)) {
				return true;
			}
			if ((int)(x2 * eps) >= (int)(obstacles.at(i).aX * eps) && (int)(x2 * eps) <= (int)(obstacles.at(i).dX * eps) && (int)(y2 * eps) >= (int)(obstacles.at(i).aY * eps) && (int)(y2 * eps) <= (int)(obstacles.at(i).dY * eps) &&
				(int)(x2 * eps) >= (int)(newInitial.x * eps) && (int)(x2 * eps) <= (int)(newTarget.x * eps) && (int)(y2 * eps) >= (int)(newInitial.y * eps) && (int)(y2 * eps) <= (int)(newTarget.y * eps)) {
				return true;
			}
			if ((int)(x3 * eps) >= (int)(obstacles.at(i).aX * eps) && (int)(x3 * eps) <= (int)(obstacles.at(i).dX * eps) && (int)(y3 * eps) >= (int)(obstacles.at(i).aY * eps) && (int)(y3 * eps) <= (int)(obstacles.at(i).dY * eps) &&
				(int)(x3 * eps) >= (int)(newInitial.x * eps) && (int)(x3 * eps) <= (int)(newTarget.x * eps) && (int)(y3 * eps) >= (int)(newInitial.y * eps) && (int)(y3 * eps) <= (int)(newTarget.y * eps)) {
				return true;
			}
			if ((int)(x4 * eps) >= (int)(obstacles.at(i).aX * eps) && (int)(x4 * eps) <= (int)(obstacles.at(i).dX * eps) && (int)(y4 * eps) >= (int)(obstacles.at(i).aY * eps) && (int)(y4 * eps) <= (int)(obstacles.at(i).dY * eps) &&
				(int)(x4 * eps) >= (int)(newInitial.x * eps) && (int)(x4 * eps) <= (int)(newTarget.x * eps) && (int)(y4 * eps) >= (int)(newInitial.y * eps) && (int)(y4 * eps) <= (int)(newTarget.y * eps)) {
				return true;
			}
		}
		else if (obstacles.at(i).type == 1) {
			if (target.x > obstacles.at(i).dX || target.x < obstacles.at(i).aX) {
				return true;
			}
			if (target.y > obstacles.at(i).dY || target.y < obstacles.at(i).aY) {
				return true;
			}
		}
	}
	return false;
}

//Get a random state from the map
state generateRandomState(vector<obstacle> obstacles, vector<state> chosen) {

	random_device rd; 
	mt19937 gen1(rd());
	mt19937 gen2(rd());
	mt19937 gen3(rd());
	state randState;

	obstacle mapBoundary;
	for (int i = 0; i < obstacles.size(); i++) {
		if (obstacles.at(i).type == 1) {
			mapBoundary = obstacles.at(i);
			break;
		}
	}
	uniform_real_distribution<double> xRand(mapBoundary.aX, mapBoundary.dX);
	uniform_real_distribution<double> yRand(mapBoundary.aY, mapBoundary.dY);
	uniform_real_distribution<double> thetaRand(0, 2*M_PI);

	randState.x = xRand(gen1);
	randState.y = yRand(gen2);
	randState.theta = thetaRand(gen3);

	//std::cout << "RANDOM STATE: " << randState.x << " " << randState.y << " " << randState.theta << "\n";
	bool failed = true;

	while (failed == true) {
		for (int i = 0; i < chosen.size(); i++) {
			if (abs(randState.x - chosen.at(i).x) < 0.00001 && abs(randState.y - chosen.at(i).y) < 0.00001 && abs(randState.theta - chosen.at(i).theta) < 0.00001) {
				randState.x = xRand(gen1);
				randState.y = yRand(gen2);
				randState.theta = thetaRand(gen3);
				i = -1;
			}
		}

		bool failCheck = true;
		for (int i = 0; i < obstacles.size(); i++) {
			if (randState.x >= obstacles.at(i).aX && randState.y >= obstacles.at(i).aY && randState.x <= obstacles.at(i).dX && randState.y <= obstacles.at(i).dY && obstacles.at(i).type == 0) {
				randState.x = xRand(gen1);
				randState.y = yRand(gen2);
				randState.theta = thetaRand(gen3);
				i = -1;
				failCheck = false;
			}
		}
		if (failCheck == true) {
			failed = false;
		}
	}

	return randState;
}

//functor for the multithread
void threadedNearestNeighbor(vector<state> &resultNearest, vector<state> partialGraph, state randState) {

	resultNearest = nearestNeighbors(partialGraph, randState, 1);
}

//generate RRT
vector<vector<state>> generateRRT(vector<obstacle> obstacles, state initial, state target) {
	vector<vector<state>> graph;
	vector<state> col;
	col.push_back(initial);
	graph.push_back(col);

	//Max limit
	const int ITERATIONS = 1000;

	vector<state> chosenStates;
	chosenStates.push_back(initial);
	chosenStates.push_back(target);
	int count = 0;
	while(true && count < ITERATIONS) {
		state randState = generateRandomState(obstacles, chosenStates);
		vector<state> candidates;
		vector<state>* resultNearest = new vector<state>[graph.size()];
		thread* run = new thread[graph.size()];

		//find nearest neighbor for the random state
		for (int k = 0; k < graph.size(); k++) {
			run[k] = thread(threadedNearestNeighbor,std::ref(resultNearest[k]), graph.at(k), randState);
		}

		//put the results together
		for (int k = 0; k < graph.size(); k++) {
			run[k].join();
			candidates.push_back(resultNearest[k].at(0));
		}
		
		delete[] resultNearest;
		delete[] run;

		//pick the nearest from the results
		vector<state> nearest = nearestNeighbors(candidates, randState, 1);
		state near = nearest.at(0);

		//get trajectory from the nearest node in graph to the random state
		vector<trajectory> resultTrajectory = generateTrajectory(near, randState);
		bool collided = false;
		bool addedOnce = false;
		for (int j = 0; j < resultTrajectory.size() - 1; j++) {
			//check collision
			collided = collisionCheck(obstacles, resultTrajectory.at(j).finalState, resultTrajectory.at(j + 1).finalState);
			if (collided == true) {
				break;
			}
			else {
				//if no collision in the trajectory, add that state from the trajectory path to the graph
				//the trajectory returns 5 states
				int verticeIndex;
				for (int k = 0; k < graph.size(); k++) {
					if (graph.at(k).at(0).x == resultTrajectory.at(j).finalState.x && graph.at(k).at(0).y == resultTrajectory.at(j).finalState.y && graph.at(k).at(0).theta == resultTrajectory.at(j).finalState.theta) {
						verticeIndex = k;
						break;
					}
				}

				//if one of the 5 states are already in the graph dont add it
				bool found = false;
				for (int k = 0; k < graph.at(verticeIndex).size(); k++) {
					if (graph.at(verticeIndex).at(k).x == resultTrajectory.at(j+1).finalState.x && graph.at(verticeIndex).at(k).y == resultTrajectory.at(j+1).finalState.y && graph.at(verticeIndex).at(k).theta == resultTrajectory.at(j+1).finalState.theta) {
						found = true;
						break;
					}
				}
				if (found == false) {
					addedOnce = true;
					graph.at(verticeIndex).push_back(resultTrajectory.at(j + 1).finalState);
						
					vector<state> newCol;
					newCol.push_back(resultTrajectory.at(j + 1).finalState);
					chosenStates.push_back(resultTrajectory.at(j + 1).finalState);
					graph.push_back(newCol);
				}
			}
		}

		//if all the points from the trajectory were added, check if it can reach the final state
		//if there is a path without disruption then return the graph
		bool linked = false;
		if (collided == false) {

			resultTrajectory = generateTrajectory(randState, target);
			collided = false;
			for (int j = 0; j < resultTrajectory.size() - 1; j++) {
				collided = collisionCheck(obstacles, resultTrajectory.at(j).finalState, resultTrajectory.at(j + 1).finalState);
				if (collided == true) {
					break;
				}
			}

			if (collided == false) {
				for (int j = 0; j < resultTrajectory.size() - 1; j++) {
					int verticeIndex;
					for (int k = 0; k < graph.size(); k++) {
						if (graph.at(k).at(0).x == resultTrajectory.at(j).finalState.x && graph.at(k).at(0).y == resultTrajectory.at(j).finalState.y && graph.at(k).at(0).theta == resultTrajectory.at(j).finalState.theta) {
							verticeIndex = k;
							break;
						}
					}

					bool found = false;
					for (int k = 1; k < graph.at(verticeIndex).size(); k++) {
						if (graph.at(verticeIndex).at(k).x == resultTrajectory.at(j + 1).finalState.x && graph.at(verticeIndex).at(k).y == resultTrajectory.at(j + 1).finalState.y && graph.at(verticeIndex).at(k).theta == resultTrajectory.at(j + 1).finalState.theta) {
							found = true;
							break;
						}
					}
					if (found == false) {
						addedOnce = true;
						graph.at(verticeIndex).push_back(resultTrajectory.at(j + 1).finalState);

						vector<state> newCol;
						newCol.push_back(resultTrajectory.at(j + 1).finalState);
						chosenStates.push_back(resultTrajectory.at(j + 1).finalState);
						graph.push_back(newCol);
					}
				}
				linked = true;

			}
		}
		
		count++;
		if (linked == true) {
			break;
		}
	}
	
	//print debug
	std::cout << "NUMBER OF NODES IN GRAPH: " << count << " " << graph.size() << "\n";
	for (int i = 0; i < graph.size(); i++) {
		//if (graph.at(i).size() > 1 || (graph.at(i).at(0).x == target.x && graph.at(i).at(0).y == target.y && graph.at(i).at(0).theta == target.theta)) {
			std::cout << "NODE: " << "(" << graph.at(i).at(0).x << "," << graph.at(i).at(0).y << "," << graph.at(i).at(0).theta << ")" << "->";
			for (int j = 0; j < graph.at(i).size(); j++) {
				std::cout << "(" << graph.at(i).at(j).x << "," << graph.at(i).at(j).y << "," << graph.at(i).at(j).theta << ")" << "|";
			}
			std::cout << "\n";
		//}
	}
	std::cout << "\n";
	return graph;
}

//tree traversal
void recursionHelper(vector<vector<state>> graph, stack<state> &path, vector<state> &visited, state target) {

	//break out if all the nodes have been visited or if the target state has been reached
	//this is DFS algorithm
	while (path.empty() == false && (path.top().x != target.x || path.top().y != target.y || path.top().theta != target.theta)) {
		state topElement = path.top();
		for (int i = 0; i < graph.size(); i++) {
			if (graph.at(i).at(0).x == topElement.x && graph.at(i).at(0).y == topElement.y && graph.at(i).at(0).theta == topElement.theta) {
				for (int j = 0; j < graph.at(i).size()-1; j++) {
					if (graph.at(i).size() == 1) {
						if (path.top().x != target.x && path.top().y != target.y && path.top().theta != target.theta) {
							path.pop();
						}
						return;
					}
					else {
						bool isVisited = false;
						state nodeToAdd = graph.at(i).at(j+1);
						
						for (int k = 0; k < visited.size(); k++) {
							if (visited.at(k).x == nodeToAdd.x && visited.at(k).y == nodeToAdd.y && visited.at(k).theta == nodeToAdd.theta) {
								isVisited = true;
								break;
							}
						}

						if (isVisited == false) {
							path.push(nodeToAdd);
							visited.push_back(nodeToAdd);
							recursionHelper(graph, path, visited, target);
							
						}
					}
				}
				if (path.top().x != target.x && path.top().y != target.y && path.top().theta != target.theta) {
					path.pop();
				}
			}
		}
	}
}

//find the path
void findPath(vector<vector<state>> graph, state initial, state target) {
	stack<state> pathInvert;
	pathInvert.push(initial);
	vector<state> visited;
	visited.push_back(initial);

	stack<state> pathAnswer;
	recursionHelper(graph, pathInvert, visited, target);

	while (pathInvert.empty() == false) {
		pathAnswer.push(pathInvert.top());
		pathInvert.pop();
	}
	while (pathAnswer.empty() == false) {
		cout << "(" << pathAnswer.top().x << "," << pathAnswer.top().y << "," << pathAnswer.top().theta << ")";
		pathAnswer.pop();
		if (pathAnswer.empty() == false) {
			cout << "->";
		}
	}
	cout << "\n";
}

int main(int argc, char** argv) {
	vector<state> V;
	vector<obstacle> obstacles;
	state target;
	state initial;
	state temp;

	/*target.x = 1;
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
	cout << "Nearest Neighbors: " << "\n";
	for (int i = 0; i < resultNearest.size(); i++) {
		cout << resultNearest.at(i).x << " " << resultNearest.at(i).y << "\n";
	}
	cout << "\n";*/
	target.x = 83;
	target.y = 7;
	target.theta = M_PI/3;

	initial.x = 50;
	initial.y = 35;
	initial.theta = 3 * M_PI / 2;
	
	/*vector<trajectory> resultTrajectory = generateTrajectory(initial, target);
	cout << "Get Trajectory: " << "\n";
	for (int i = 0; i < resultTrajectory.size(); i++) {
		cout << resultTrajectory.at(i).finalState.x << " " << resultTrajectory.at(i).finalState.y << " "
			<< resultTrajectory.at(i).inputWheelLeft << " " << resultTrajectory.at(i).inputWheelRight << " "
			<< resultTrajectory.at(i).time << "\n";
	}
	cout << "\n";*/
	obstacle obs;
	obs.aX = 10;
	obs.aY = 0;
	obs.dX = 90;
	obs.dY = 5;
	obs.type = 0;
	obstacles.push_back(obs);

	obs.aX = 10;
	obs.aY = 5;
	obs.dX = 80;
	obs.dY = 10;
	obs.type = 0;
	obstacles.push_back(obs);

	obs.aX = 85;
	obs.aY = 5;
	obs.dX = 90;
	obs.dY = 10;
	obs.type = 0;
	obstacles.push_back(obs);

	obs.aX = 10;
	obs.aY = 20;
	obs.dX = 90;
	obs.dY = 30;
	obs.type = 0;
	obstacles.push_back(obs);

	obs.aX = 10;
	obs.aY = 40;
	obs.dX = 90;
	obs.dY = 50;
	obs.type = 0;
	obstacles.push_back(obs);

	obs.aX = 0;
	obs.aY = 0;
	obs.dX = 100;
	obs.dY = 50;
	obs.type = 1;
	obstacles.push_back(obs);
	/*cout << "Collision Detection: " << "\n";
	for(int i = 0; i < resultTrajectory.size()-1; i++){
		cout << resultTrajectory.at(i).finalState.x << " " << resultTrajectory.at(i).finalState.y << " " << collisionCheck(obstacles, resultTrajectory.at(i).finalState, resultTrajectory.at(i + 1).finalState) << "\n";
	}
	cout << "\n";*/

	cout << "INITIAL: " << "\n";
	cout << initial.x << " " << initial.y << " " << initial.theta << "\n";

	cout << "TARGET: " << "\n";
	cout << target.x << " " << target.y << " " << target.theta << "\n";

	cout << "OBSTACLES: " << "\n";
	for (int i = 0; i < obstacles.size(); i++) {
		if (i == obstacles.size() - 1) {
			cout << "MAP SIZE: ";
		}
		cout << "((" << obstacles.at(i).aX << "," << obstacles.at(i).aY << ")(" << obstacles.at(i).dX << "," << obstacles.at(i).dY << "))" << "\n";
	}
	cout << "\n";

	high_resolution_clock::time_point t1;
	high_resolution_clock::time_point t2; 
	duration<double> time_span; 

	cout << "Generate RRT: " << "\n";
	t1 = high_resolution_clock::now();
	vector<vector<state>> graph = generateRRT(obstacles, initial, target);

	t2 = high_resolution_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
	std::cout << "Time to execute RRT: " << time_span.count() << " seconds" << "\n";

	cout << "Running graph search: " << "\n";
	t1 = high_resolution_clock::now();
	findPath(graph, initial, target);
	t2 = high_resolution_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
	std::cout << "Time to execute DFS: " << time_span.count() << " seconds" << "\n";
}