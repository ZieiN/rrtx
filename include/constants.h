//
// Created by zain on 09.07.2021.
//

#ifndef RRTX_CONSTANTS_H
#define RRTX_CONSTANTS_H

#include <cmath>
#include <string>
using namespace  std;

//bool DEBUG = true;
//bool REPLANNING = false;
//string testFileName = "1";
//const double M_PI = 3.141592653;
const double INF = 1e9;
const double EPS_GOAL = 1;
const double MX_HEIGHT = 600;
const double MX_WIDTH = 600;
const double DELTA = 60;
const double EPS = 0;
const double EPS_DOUBLE = 1e-9;
const double EPS_ORIENTATION_ANGLE = 0.1;
const double DIM = 2;
const double MU_OBSTACLES = MX_HEIGHT * MX_WIDTH;
const double ZETA = M_PI;
const double GAMMA = 2 * pow((1 + DIM) * (MU_OBSTACLES / ZETA), 1 / DIM);
//const int RAND_SEED = 100;
const double probability_SAMPLE_START_NODE = 0.3;


// Dubins
const double VELOCITY = 5;
const double MIN_RADIUS = 10;
const double WHEEL_BASE = 3.25;
const double twopi = 2. * M_PI;
const double DUBINS_EPS = 1e-6;
const double DUBINS_ZERO = -1e-7;

#endif //RRTX_CONSTANTS
