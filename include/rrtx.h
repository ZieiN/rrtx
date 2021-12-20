//
// Created by zain on 09.07.2021.
//

#ifndef RRTX_RRTX_H
#define RRTX_RRTX_H

#include "constants.h"
#include "map.h"
#include "point.h"
#include "../include/Dubins/dubins.h"
#include <bits/stdc++.h>

using namespace std;


class Node {
public:
    Point pnt_;
    double g_, lmc_;
    Node *selfItr_;
    Node *parent_;
    set<Node *> NPlus0_, NPlusR_;
    set<Node *> NPlusNotConnected0_, NPlusNotConnectedR_;
    set<Node *> NMinus0_, NMinusR_, children_;
    set<Node *> NMinusNotConnected0_, NMinusNotConnectedR_;

    Node() {
        children_.clear();
        NMinus0_.clear();
        NPlus0_.clear();
        NMinusR_.clear();
        NPlusR_.clear();
        NPlusNotConnected0_.clear();
        NPlusNotConnectedR_.clear();
        NMinusNotConnected0_.clear();
        NMinusNotConnectedR_.clear();
        g_ = lmc_ = INF;
    }

    Node(Point pnt, double g, double lmc, Node *selfItr, Node *parent) :
            pnt_(pnt), g_(g), lmc_(lmc), selfItr_(selfItr), parent_(parent) {
        children_.clear();
        NMinus0_.clear();
        NPlus0_.clear();
        NMinusR_.clear();
        NPlusR_.clear();
        NPlusNotConnected0_.clear();
        NPlusNotConnectedR_.clear();
        NMinusNotConnected0_.clear();
        NMinusNotConnectedR_.clear();

    }
};
bool operator<(pair<pair<double, double>, Node *> a, pair<pair<double, double>, Node *> b);

class RRTX{
public:
    RRTX();
    RRTX(Point, Point);

    Map mp_;
    list<Node> nodes_;
    double radius_;
    Node *vBot_;
    bool vBotIsAdded_;
    Point startPoint_, goal_;
    set<Node *> orphans_;
    set<Node *> deletedNodes_;
    priority_queue<pair<pair<double, double>, Node *>> qRewiring_;


    bool extend(Node &v);
    list<Node *> near(Node &v);
    void findParent(Node &v, list<Node *> &vNear);
    static double distanceFunction(Point &v1, Point &v2);
    double distanceBySteerFunction(Point &v1, Point &v2);
    bool steer(Point v1, Point &v2);
    void steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory);
    void updateMap(Map &mp);
    void updateMapFromFile(string fileName);
    void addAllChildren(Node *vItr);
    void propagateDescendants();
    void initializeTree();
    void reduceInconsistency();
    void updateLMC(Node *vItr);
    void rewireNeighbors(Node *&vItr);
    void cullNeighbors(Node *&vItr);
    void verifyQueue(Node *vItr);
    void saturate(Node &v, Node *&vNearest);
    Node *nearest(Node &v);
    Point randomNode();
    void shrinkingBallRadius();
    bool search();
    void deleteNode(list<Node>::iterator &pntNode);
    void shiftTree(double, double);
    void drawSolution(string fileName);
    void drawTree(string fileName);
    void check();
    void moveRobot(double shiftDeltaX, double shiftDeltaY);
    void shiftGoal(double shiftDeltaX, double shiftDeltaY);
    void checkForDisappearedObstacles();
    void checkForAppearedObstacles();
    void moveObstacles(string fileName, int shift);

    vector<Point> getPath();
    bool achievedGoalState(Node);

    void drawMap(string fileName);

    void addDynamicObstacles(string fileName, int shift);
};

#endif //RRTX_RRTX_H
