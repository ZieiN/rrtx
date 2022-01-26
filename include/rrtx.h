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
    set<pair<Node *, DubinsPath>> NPlus0_, NPlusR_;
    set<pair<Node *, DubinsPath>> NPlusNotConnected0_, NPlusNotConnectedR_;
    set<pair<Node *, DubinsPath>> NMinus0_, NMinusR_;
    set<pair<Node *, DubinsPath>> NMinusNotConnected0_, NMinusNotConnectedR_;
    set<Node *> children_;
    DubinsPath pathToParent_;

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
//bool operator<(pair<pair<double, double>, Node *> a, pair<pair<double, double>, Node *> b);
bool operator < (DubinsPath dp1, DubinsPath dp2);
bool operator < (std::_List_iterator<std::pair<std::pair<Node*, Node*>, std::pair<DubinsPath, pair<bool, int> >> >, std::_List_iterator<std::pair<std::pair<Node*, Node*>, pair<DubinsPath, std::pair<bool, int> > > >);
class RRTX{
public:
    RRTX();
    RRTX(Point, Point);

    Map mp_;
    list<Node> nodes_;
    double radius_;
    Dubins dubins_;
    Node *vBot_;
    bool vBotIsAdded_;
    Point startPoint_, goal_;
    set<Node *> orphans_;
    set<Node *> deletedNodes_;
    priority_queue<pair<pair<double, double>, Node *>> qRewiring_;
    bool flagPlanNearOldPath;
    vector<Point> guidePath;
    vector<Point> finalPath;
//    vector<bool> isValidEdge;
    map<int, map<int, set<_List_iterator<pair<pair<Node *, Node *>, pair<DubinsPath, pair<bool, int>>>>>>> passedEdges;
    list<pair<pair<Node *, Node *>, pair<DubinsPath, pair<bool, int>>>> edges; // <from, to>, <dubinsPath, <R or 0, valid/invalid>>
    bool updatePathNeeded;
    bool guidePathDefined;
    int counterAddPath;
    int shiftX_, shiftY_;





    Node *nearest(Node &v);
    Point randomNode();
    bool achievedStartState(Point);
    bool checkDubinsPath(const DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy, double t1, double t2);
    bool checkDubinsPath(const DubinsPath &dubinsPath, Point &p0, Point &p1);
    bool extend(Node &v);
    bool search();

    double distanceBySteerFunction(Point &v1, Point &v2);

    list<pair<Node *, DubinsPath>> near(Node &v);

    pair<bool, DubinsPath> steer(Point v1, Point &v2);

    static double distanceFunction(Point &v1, Point &v2);

    void addAllChildren(Node *vItr);
    void addEdge(Node *p1, pair<Node *, DubinsPath> p2, bool direction);
    void check();
    void checkAllEdges();
    void cullNeighbors(Node *&vItr);
    void deleteNode(list<Node>::iterator &pntNode);
    void drawMap(string fileName);
    void drawSolution(string fileName);
    void drawTree(string fileName);
    void findParent(Node &v, list<pair<Node *, DubinsPath>> &vNear);
    void getLatticePointsOnThePath(DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy, double t1, double t2, set<pair<int, int>> &pathPoints);
    void initializeTree();
    void moveObstacles(string fileName, int shift);
    void moveRobot(int shiftDeltaX, int shiftDeltaY, double newTheta);
    void propagateDescendants();
    void reduceInconsistency();
    void resetTreeFillGuidePath();
    void rewireNeighbors(Node *&vItr);
    void saturate(Node &v, Node *&vNearest);
    void shiftGoal(int shiftDeltaX, int shiftDeltaY);
    void shiftTree(int, int);
    void shrinkingBallRadius();
    void steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory, DubinsPath &dubinsPath, double step);
    void steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory, double step);
    void updateEdges();
    void updateLMC(Node *vItr);
    void updateMap(Map &mp);
    void updateMapFromFile(string fileName);
    void updatePath(double step);
    void verifyQueue(Node *vItr);
//    bool checkPartOfLeftCircle(double cntrX, double cntrY, double r, double stx, double sty, double ndx, double ndy);
//    void checkForAppearedObstacles();
//    void checkForDisappearedObstacles();
};

#endif //RRTX_RRTX_H
