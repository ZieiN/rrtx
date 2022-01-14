//
// Created by zain on 09.07.2021.
//
#ifndef RRTX_RRTX_CPP
#define RRTX_RRTX_CPP

#include "../include/rrtx.h"
#include "../include/Simplifier/simplifymax.h"
#include <bits/stdc++.h>

using namespace std;
using namespace std::chrono;

RRTX::RRTX() {
    mp_ = Map();
    nodes_.clear();
    vBotIsAdded_ = false;
    updatePathNeeded = false;
    flagPlanNearOldPath = false;
    guidePathDefined = false;
}

RRTX::RRTX(Point start, Point goal) {
    mp_ = Map();
    nodes_.clear();
    vBotIsAdded_ = false;
    updatePathNeeded = false;
    flagPlanNearOldPath = false;
    guidePathDefined = false;
    startPoint_ = start;
    goal_ = goal;
    initializeTree();
}

bool operator<(pair<pair<double, double>, Node *> a, pair<pair<double, double>, Node *> b) {
    return a.first.first > b.first.first || (a.first.first == b.first.first && a.first.second > b.first.second);
}

Point RRTX::randomNode() {
//    double probability = rand() / (double) RAND_MAX;
//    if (probability < probability_SAMPLE_START_NODE) {
//        return startPoint_;
//    }
//    double x = rand() / (double) RAND_MAX * MX_HEIGHT;
//    double y = rand() / (double) RAND_MAX * MX_WIDTH;
//    double theta = rand() / (double) RAND_MAX * 2*M_PI;
//    return Point(x, y, theta);
    if(guidePathDefined && counterAddPath < guidePath.size()){
        counterAddPath += 10;
        return guidePath[(int)(guidePath.size())-1-counterAddPath-10];
    }
     random_device rd;
//    static int cnt = 0;
     mt19937 mt(rd());
    uniform_real_distribution<double> randForStartPoint(0, 1);
    if (randForStartPoint(mt) < probability_SAMPLE_START_NODE) {
        return startPoint_;
    }
     if(guidePathDefined){
         uniform_int_distribution<int> randPntPath(0,(int)guidePath.size()-1);
         uniform_real_distribution<double> randAng(0, 2*M_PI);
         uniform_real_distribution<double> randRadius(0, DELTA2);
         int id = randPntPath(mt);
         double rad = randRadius(mt);
         double ang = randAng(mt);
         double x = guidePath[id].x_+rad*cos(ang);
         double y = guidePath[id].y_+rad*sin(ang);
         return Point(x, y, ang);
     }
     else {
         uniform_real_distribution<double> randHeight(0, MX_HEIGHT);
         uniform_real_distribution<double> randWidth(0, MX_WIDTH);
         uniform_real_distribution<double> randOrientation(0, 2 * M_PI);
         uniform_real_distribution<double> randForStartPoint(0, 1);
         return Point(randWidth(mt), randHeight(mt), randOrientation(mt));
     }
}

void RRTX::shrinkingBallRadius() {
    radius_ = min(GAMMA * pow(log(nodes_.size() + 1) / (1.0 + nodes_.size()), 1.0 / DIM), DELTA);
}

bool RRTX::extend(Node &v) {
    list<Node *> vNear = near(v);
    findParent(v, vNear);
    if (v.parent_ == NULL) {
        return false;
    }
    auto it = nodes_.insert(nodes_.end(), v);
    Node *vItr = &(*it);
    vItr->selfItr_ = v.selfItr_ = vItr;
//    assert(&(*vItr) == vItr->selfItr_);
    if(deletedNodes_.find(vItr->selfItr_)!=deletedNodes_.end()){
        deletedNodes_.erase(vItr->selfItr_);
    }
    v.parent_->children_.insert(vItr);
    for (auto &it: vNear) {
        if (steer(vItr->pnt_, it->pnt_)) {
            vItr->NPlus0_.insert(it->selfItr_);
            (it->selfItr_)->NMinusR_.insert(vItr);
        }
        if (steer(it->pnt_, v.pnt_)) {
            (it->selfItr_)->NPlusR_.insert(vItr->selfItr_);
            vItr->NMinus0_.insert(it->selfItr_);
        }
    }
    return true;
}

list<Node *> RRTX::near(Node &v) {
    list<Node *> ans;
    for (auto &it: nodes_) {
        /// because I want to add the orphans to 'near'-set so they can be rewired.
        // if(it.parent_ == NULL)
        //     continue;
        if (distanceFunction(v.pnt_, it.pnt_) <= radius_) {
            if(steer(v.pnt_, it.pnt_)) {
                ans.push_back(it.selfItr_);
            }
        }
    }
    return ans;
}

void RRTX::findParent(Node &v, list<Node *> &vNear) {
    // we prune the neighbor set here, in addition to find the parent.
    if(vNear.empty()) {
        return;
    }
    vector<pair<double, Node*>> vectorNeighbors;
    for (auto &it: vNear) {
        double costTransition = distanceBySteerFunction(v.pnt_, it->pnt_);
        vectorNeighbors.emplace_back(costTransition+it->lmc_, it);
//        assert(steer(v.pnt_, it->pnt_));
//        if (costTransition + it->lmc_ < v.lmc_) {
//            v.parent_ = it->selfItr_;
//            v.lmc_ = costTransition + it->lmc_;
//        }
    }
    sort(vectorNeighbors.begin(), vectorNeighbors.end());
    if(vNear.size()>MAX_NUM_NEIGHNORS){
        vNear.clear();
        for(int i=0; i<MAX_NUM_NEIGHNORS; ++i){
            vNear.push_back(vectorNeighbors[i].second);
        }
    }
//    cout<<v.pnt_.x_<<","<<v.pnt_.y_<<","<<v.pnt_.theta_<<"::";
//    for(auto it:vNear){
//        cout<<it->pnt_.x_<<","<<it->pnt_.y_<<" ";
//        assert(steer(v.pnt_, it->pnt_));
//    }cout<<endl;
    if(vectorNeighbors[0].first < v.lmc_) {
        v.parent_ = vectorNeighbors[0].second->selfItr_;
        v.lmc_ = vectorNeighbors[0].first;
    }
}

double RRTX::distanceFunction(Point &v1, Point &v2) {
    return hypot(v1.x_ - v2.x_, v1.y_ - v2.y_);
}

bool RRTX::steer(Point v1, Point &v2) {
    auto startTime = high_resolution_clock::now();
    Dubins d(MIN_RADIUS, 0);
    auto path=d.dubins(&v1, &v2);
    return checkDubinsPath(d, path, &v1, v1.x_, v1.y_, v2.x_, v2.y_, 0, 1);
    vector<Point> v;
    int numOfTimeSteps = d.distance(&v1, &v2)+1;
//    cerr<<numOfTimeSteps<<endl;
    double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//    cerr << "Time to create dubins " << diffTime / 1000.0 << "ms" << endl;
    for(int i=1; i<=numOfTimeSteps; ++i){
        Point p;
        d.interpolate(&v1, path, (double)i/numOfTimeSteps, &p);
        if(mp_.cellIsObstacle(p.x_, p.y_)) {

            double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//            cerr << "Time to update steer " << diffTime / 1000.0 << "ms" << endl;
            return false;
        }
//        v.push_back(p);
    }

    diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//    cerr << "Time to update the steerBig " << diffTime/1000.0 << "ms" << endl;
    return true;
//    AgentState startState, goal;
//    startState.pos = {v1.x_, v1.y_};
//    startState.theta = v1.theta_;
//    goal.pos = {v2.x_, v2.y_};
//    goal.theta = v2.theta_;
//    mp_.agent_.SetState(startState);
//    mp_.agent_.SetGoal(goal);
//    vector<AgentState> v;
//    while(mp_.agent_.Update(v)){
//        if(mp_.cellIsObstacle(v.back().pos.first, v.back().pos.second))
//            return false;
//    }
//    return true;
}


void RRTX::checkForAppearedObstacles(){
//    int sm = nodes_.size();
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
//        sm += it->NPlusR_.size() + it->NPlus0_.size();
//        auto startTime = high_resolution_clock::now();
        for (auto it1 = it->NPlus0_.begin(); it1 != it->NPlus0_.end();) {
            if (!steer(it->pnt_, (*it1)->pnt_)) {
//                auto startTime = high_resolution_clock::now();
                it->NPlusNotConnected0_.insert(*it1);
                (*it1)->NMinusNotConnectedR_.insert(&*it);
                if (it->parent_ == (*it1)) {
                    orphans_.insert(&*it);
                    it->g_ = INF;
                    (*it1)->children_.erase(&*it);
                    it->parent_ = NULL;
                }
                (*it1)->NMinusR_.erase(&*it);
                it1 = (it->NPlus0_).erase(it1);
//                auto diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//                cerr << "Time to delete0 " << diffTime/1000.0 << "ms" << endl;
            } else {
                ++it1;
            }
        }
//
//        double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//        cerr << "Time to update the map3-1 " << diffTime/1000.0 << "ms" << endl;
//        startTime = high_resolution_clock::now();
        for (auto it1 = it->NPlusR_.begin(); it1 != it->NPlusR_.end();) {
            if (!steer(it->pnt_, (*it1)->pnt_)) {

//                auto startTime = high_resolution_clock::now();
                it->NPlusNotConnectedR_.insert(*it1);
                (*it1)->NMinusNotConnected0_.insert(&*it);
                if (it->parent_ == (*it1)) {
                    orphans_.insert(&*it);
                    it->g_ = INF;
                    (*it1)->children_.erase(&*it);
                    it->parent_ = NULL;
                }
                (*it1)->NMinus0_.erase(&*it);
                it1 = (it->NPlusR_).erase(it1);
//                auto diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//                cerr << "Time to delete1 " << diffTime/1000.0 << "ms" << endl;
            } else {
                ++it1;
            }
        }
//
//        diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
////        cerr << "Time to update the map3-2 " << diffTime/1000.0 << "ms" << endl;
//        startTime = high_resolution_clock::now();
//        for (auto it1 = it->NMinus0_.begin(); it1 != it->NMinus0_.end();) {
//            if (!steer((*it1)->pnt_, it->pnt_)) {
//                startTime = high_resolution_clock::now();
//                it->NMinusNotConnected0_.insert(*it1);
//                (*it1)->NPlusNotConnectedR_.insert(&*it);
//                if ((*it1)->parent_ == it->selfItr_) {
//                    orphans_.insert(*it1);
//                    (*it1)->g_ = INF;
//                    (*it1)->parent_ = NULL;
//                    it->children_.erase((*it1)->selfItr_);
//                }
//                (*it1)->NPlusR_.erase(&*it);
//                it1 = (it->NMinus0_).erase(it1);
//                diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
//                cerr << "Time to delete " << diffTime/1000.0 << "ms" << endl;
//            } else {
//                ++it1;
//            }
//        }
//
//        diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
////        cerr << "Time to update the map3-3 " << diffTime/1000.0 << "ms" << endl;
//        startTime = high_resolution_clock::now();
//        for (auto it1 = it->NMinusR_.begin(); it1 != it->NMinusR_.end();) {
//            if (!steer((*it1)->pnt_, it->pnt_)) {
//                it->NMinusNotConnectedR_.insert(*it1);
//                (*it1)->NPlusNotConnected0_.insert(&*it);
//                if ((*it1)->parent_ == it->selfItr_) {
//                    orphans_.insert(*it1);
//                    (*it1)->g_ = INF;
//                    (*it1)->parent_ = NULL;
//                    it->children_.erase((*it1)->selfItr_);
//                }
//                (*it1)->NPlus0_.erase(&*it);
//                it1 = (it->NMinusR_).erase(it1);
//            } else {
//                ++it1;
//            }
//        }

//        diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
////        cerr << "Time to update the map3-4 " << diffTime/1000.0 << "ms" << endl;
//        startTime = high_resolution_clock::now();
    }
//    cout<<"SUM = "<<sm<<endl;
}

void RRTX::addAllChildren(Node *vItr) {
//    assert(deletedNodes_.find(vItr)==deletedNodes_.end());
    for (auto &it: vItr->children_) {
//        assert(orphans_.find(it)==orphans_.end());
        orphans_.insert(it);
        addAllChildren(it);
    }
}

void RRTX::propagateDescendants() {
    auto cpyOrphans = orphans_;
    for (auto &it: cpyOrphans) {
        addAllChildren(it);
    }
    for (auto &it: orphans_) {
        for (auto it1: it->NPlusR_) {
            it1->g_ = INF;
            verifyQueue(it1);
        }
        for (auto it1: it->NPlus0_) {
            it1->g_ = INF;
            verifyQueue(it1);
        }
        if (vBotIsAdded_) {
            if (vBot_ == it) {
                vBotIsAdded_ = false;
                updatePathNeeded = true;
            }
        }
    }
    for (auto it = orphans_.begin(); it != orphans_.end();) {
        (*it)->g_ = INF;
        (*it)->lmc_ = INF;
        if ((*it)->parent_ != NULL) {
//            assert(((*it)->parent_)->children_.find((*it)->selfItr_) != ((*it)->parent_)->children_.end());
            ((*it)->parent_)->children_.erase((*it)->selfItr_);
            (*it)->parent_ = NULL;
        }
        auto it1 = it;
        ++it;
        orphans_.erase(it1);
    }
//    assert(orphans_.size()==0);
}


void RRTX::drawSolution(string fileName) {
    ofstream out;
    out.open(fileName.c_str());
    if(updatePathNeeded){
        updatePath();
    }
    for(int i=0; i+1<finalPath.size(); ++i){
//        assert(mp_.cellIsObstacle(finalPath[i].x_, finalPath[i].y_)==0);
        out<<finalPath[i].x_<<" "<<finalPath[i].y_<<" "<<finalPath[i+1].x_<<" "<<finalPath[i+1].y_<<endl;
    }
    out.close();
}

void RRTX::cullNeighbors(Node *&vItr) {
    for (auto it = vItr->NPlusR_.begin(); it != vItr->NPlusR_.end();) {
//        assert(*it != NULL);
        if ((*it)->selfItr_ == vItr->parent_ || distanceFunction((*it)->pnt_, vItr->pnt_)<=radius_) {
            ++it;
            continue;
        }
        auto it1 = it;
        ++it;
        (*it1)->NMinus0_.erase(vItr);
        vItr->NPlusR_.erase(it1);
    }
}

// Search the nodes around me (vItr) which I can connect through them, and try to improve my lmc through them.
void RRTX::updateLMC(Node *vItr) {
    cullNeighbors(vItr);
    for (auto &it: vItr->NPlus0_) {
        if (it->parent_ == vItr) {
            continue;
        }
        if (/* steer(vItr->pnt_, it->pnt_) &&*/
        vItr->lmc_ > distanceBySteerFunction(vItr->pnt_, it->pnt_) + it->lmc_ && orphans_.find(it)==orphans_.end()) {
            vItr->lmc_ = distanceBySteerFunction(vItr->pnt_, it->pnt_) + it->lmc_;
            if (vItr->parent_ != NULL)
                (vItr->parent_)->children_.erase(vItr->selfItr_);
            vItr->parent_ = it->selfItr_;
            (vItr->parent_)->children_.insert(vItr->selfItr_);
        }
    }
    for (auto &it: vItr->NPlusR_) {
        if (it->parent_ == vItr) {
            continue;
        }
        if (/*steer(vItr->pnt_, it->pnt_) && */
        vItr->lmc_ > distanceBySteerFunction(vItr->pnt_, it->pnt_) + it->lmc_ && orphans_.find(it)==orphans_.end()) {
            vItr->lmc_ = distanceBySteerFunction(vItr->pnt_, it->pnt_) + it->lmc_;
            if (vItr->parent_ != NULL)
                (vItr->parent_)->children_.erase(vItr->selfItr_);
            vItr->parent_ = it->selfItr_;
            (vItr->parent_)->children_.insert(vItr->selfItr_);
        }
    }
}


// For all nodes in the queue, if their difference (g-lmc) is bigger than delta,
// then update the lmc for the best value and then assign it to g_.
void RRTX::reduceInconsistency() {
    while (!qRewiring_.empty()) {
        auto tp = qRewiring_.top();
        qRewiring_.pop();
        //check if this node is erased before
        if(deletedNodes_.find(tp.second)!=deletedNodes_.end())
            continue;
        // check if this node is up-to-date (I don't remove old nodes when add the node with new values (maybe need to think again about this condition)).
        if (abs(min(tp.second->g_, tp.second->lmc_) - tp.first.first) > EPS_DOUBLE ||
            abs(tp.second->g_ - tp.first.second) > EPS_DOUBLE) {
            continue;
        }
        if (vBotIsAdded_) {
            if (tp.first.first > vBot_->g_) {
                vBot_->g_ = vBot_->lmc_;
                break;
            }
        }
        if (tp.second->g_ - tp.second->lmc_ > EPS) {
            updateLMC(tp.second);
            rewireNeighbors(tp.second);
        }
        tp.second->g_ = tp.second->lmc_;
    }
}

// For nodes around me (vItr), try to connect them through me if it's better.
void RRTX::rewireNeighbors(Node* &vItr) {
//    assert(orphans_.find(vItr)==orphans_.end());
    if(orphans_.find(vItr)!=orphans_.end())return;
    if (vItr->g_ - vItr->lmc_ > EPS) {
        cullNeighbors(vItr);
        for (auto &it: vItr->NMinus0_) {
            if (it->selfItr_ == vItr->parent_)
                continue;
            if (steer(it->pnt_, vItr->pnt_) && it->lmc_ > distanceBySteerFunction(it->pnt_, vItr->pnt_) + (vItr->lmc_)) {
                it->lmc_ = distanceBySteerFunction(it->pnt_, vItr->pnt_) + vItr->lmc_;
                if (it->parent_ != NULL)
                    (it->parent_)->children_.erase(it->selfItr_);
                it->parent_ = vItr->selfItr_;
                it->parent_->children_.insert(it->selfItr_);
                if (hypot(it->pnt_.x_ - startPoint_.x_, it->pnt_.y_ - startPoint_.y_) < EPS_DOUBLE) {
                    vBotIsAdded_ = true;
                    updatePathNeeded = true;
                    vBot_ = it->selfItr_;
                }
                if (it->g_ - it->lmc_ > EPS) {
                    verifyQueue(it->selfItr_);
                }
            }
        }
        for (auto &it: vItr->NMinusR_) {
            if (it->selfItr_ == vItr->parent_)
                continue;
            if (steer(it->pnt_, vItr->pnt_) && it->lmc_ > distanceBySteerFunction(it->pnt_, vItr->pnt_) + (vItr->lmc_)) {
                it->lmc_ = distanceBySteerFunction(it->pnt_, vItr->pnt_) + vItr->lmc_;
                if (it->parent_ != NULL)
                    it->parent_->children_.erase(it->selfItr_);
                it->parent_ = vItr->selfItr_;
                it->parent_->children_.insert(it->selfItr_);
                if (hypot(it->pnt_.x_ - startPoint_.x_, it->pnt_.y_ - startPoint_.y_) < EPS_DOUBLE) {
                    vBotIsAdded_ = true;
                    updatePathNeeded = true;
                    vBot_ = it->selfItr_;
                }
                if (it->g_ - it->lmc_ > EPS) {
                    verifyQueue(it->selfItr_);
                }
            }
        }
    }
}
/*
void RRTX::updateObstacles(string fileName) {
    ifstream in;
    in.open(fileName.c_str());
    int n;
    in >> n;
    double x, y, x0, y0;
    for (int i = 0; i < n; ++i) {
        in >> x0 >> y0;
        Polygon tmp(Point(x, y), {});
        int m;
        in >> m;
        for (int j = 0; j < m; ++j) {
            in >> x >> y;
            tmp.vertices_.emplace_back(x0 + x, y0 + y);
        }
        obstacles.push_back(tmp);
    }
    in.close();
    addNewObstacle();
    check();
    propagateDescendants();
    check();
    reduceInconsistency();
}
*/
bool isOut(Point &p) {
    if (p.x_ < 0 || p.x_ >= MX_WIDTH || p.y_ < 0 || p.y_ >= MX_HEIGHT)
        return true;
    return false;
}

void RRTX::check() {
    for (auto &it: nodes_) {
        assert(deletedNodes_.find(&it)==deletedNodes_.end());
        for (auto &it1: it.children_) {
            assert(it1->parent_ == it.selfItr_);
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
            assert(it.NMinus0_.find(it1)!=it.NMinus0_.end() || it.NMinusR_.find(it1)!=it.NMinusR_.end());
        }
        for (auto &it1: it.NPlusR_) {
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
            assert(it1->NMinus0_.find(it.selfItr_) != it1->NMinus0_.end());
        }
        for (auto &it1: it.NPlus0_) {
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
            assert(it1->NMinusR_.find(it.selfItr_) != it1->NMinusR_.end());
        }
        for (auto &it1: it.NMinus0_) {
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
            assert((it1->NPlusR_.find(it.selfItr_)) != it1->NPlusR_.end());
        }
        for (auto &it1: it.NMinusR_) {
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
            assert(it1->NPlus0_.find(it.selfItr_) != it1->NPlus0_.end());
        }
        if(it.parent_!=NULL) {
            if(it.parent_!=it.selfItr_) {
                assert(it.NPlusR_.find(it.parent_) != it.NPlusR_.end() ||
                       it.NPlus0_.find(it.parent_) != it.NPlus0_.end());
                assert(steer(it.pnt_, it.parent_->pnt_));
            }
        }
        assert((&(it)) == (it.selfItr_));
    }
}

void RRTX::deleteNode(list<Node>::iterator &pntNode) {
//    assert(&(*pntNode) == pntNode->selfItr_);
    // The goal node shouldn't be called to be deleted. If this happens, ignore it.
    if(pntNode==nodes_.begin()){
        return;
    }

    for (auto &it: pntNode->NPlus0_) {
        it->NMinusR_.erase(pntNode->selfItr_);
        if (pntNode->parent_ == it->selfItr_) {
//            assert(it->children_.find(pntNode->selfItr_) != it->children_.end());
            it->children_.erase(pntNode->selfItr_);
        }
    }
    for (auto &it: pntNode->NPlusR_) {
        it->NMinus0_.erase(pntNode->selfItr_);

        if (pntNode->parent_ == it->selfItr_) {
//            assert(it->children_.find(pntNode->selfItr_) != it->children_.end());
            it->children_.erase(pntNode->selfItr_);
        }
    }
    for (auto &it: pntNode->NPlusNotConnected0_) {
        it->NMinusNotConnectedR_.erase(pntNode->selfItr_);
    }
    for (auto &it: pntNode->NPlusNotConnectedR_) {
        it->NMinusNotConnected0_.erase(pntNode->selfItr_);
    }
    for (auto &it: pntNode->NMinus0_) {
//        assert(it->NPlusR_.find(pntNode->selfItr_) != it->NPlusR_.end());
        it->NPlusR_.erase(pntNode->selfItr_);
        if (it->parent_ == pntNode->selfItr_) {
            orphans_.insert(it);
            (it)->g_ = INF;
            it->parent_ = NULL;
        }
    }
    for (auto &it: pntNode->NMinusR_) {
//        assert(it->NPlus0_.find(pntNode->selfItr_) != it->NPlus0_.end());
        it->NPlus0_.erase(pntNode->selfItr_);
        if (it->parent_ == pntNode->selfItr_) {
            orphans_.insert(it);
            (it)->g_ = INF;
            it->parent_ = NULL;
        }
    }
    for (auto &it: pntNode->NMinusNotConnected0_) {
        it->NPlusNotConnectedR_.erase(pntNode->selfItr_);
    }
    for (auto &it: pntNode->NMinusNotConnectedR_) {
        it->NPlusNotConnected0_.erase(pntNode->selfItr_);
    }
    if (orphans_.find(pntNode->selfItr_) != orphans_.end())
        orphans_.erase(pntNode->selfItr_);
    if(vBotIsAdded_){
        if(vBot_==pntNode->selfItr_){
            vBot_ = NULL;
            vBotIsAdded_ = false;
            updatePathNeeded = true;
        }
    }
    deletedNodes_.insert(pntNode->selfItr_);
    nodes_.erase(pntNode);
}

void RRTX::shiftTree(double shiftDeltaX, double shiftDeltaY) {
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
//        assert(&(*it) == it->selfItr_);
        it->pnt_.x_ += shiftDeltaX;
        it->pnt_.y_ += shiftDeltaY;
//        assert((&*it) == it->selfItr_);
    }
}

void RRTX::initializeTree() {
    nodes_.clear();
    deletedNodes_.clear();
    orphans_.clear();
    Node goalNode = Node(goal_, 0, 0, NULL, NULL);
    vBotIsAdded_ = false;
    updatePathNeeded = true;
    nodes_.push_back(goalNode);
    nodes_.begin()->selfItr_ = nodes_.begin()->parent_ = &(*nodes_.begin());
    nodes_.begin()->children_.insert(nodes_.begin()->selfItr_);
    nodes_.begin()->NMinus0_.insert(nodes_.begin()->selfItr_);
    nodes_.begin()->NMinusR_.insert(nodes_.begin()->selfItr_);
    nodes_.begin()->NPlusR_.insert(nodes_.begin()->selfItr_);
    nodes_.begin()->NPlus0_.insert(nodes_.begin()->selfItr_);
    while (!qRewiring_.empty()) {
        qRewiring_.pop();
    }
}

bool RRTX::search() {
    if(!vBotIsAdded_) {
        for (auto &it: nodes_) {
            if(it.lmc_ < INF) {
                if(achievedStartState(it)){
                    vBotIsAdded_ = true;
                    vBot_ = it.selfItr_;
                    updatePath();
                    return true;
                }
            }
        }
    }
    int cnt = 0;
    auto startTime = high_resolution_clock::now();
    double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
    while (!vBotIsAdded_ && diffTime < 100000) {
//        assert(nodes_.size()>0);
        shrinkingBallRadius();
        Point randPnt = randomNode();
        Node v = Node(randPnt, INF, INF, NULL, NULL);
        Node *vNearest = nearest(v);
        if(vNearest == NULL) {
            ++cnt;
            diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
            continue;
        }
        if (distanceFunction(v.pnt_, vNearest->pnt_) > DELTA) {
            saturate(v, vNearest);
        }
        bool flagAdded = false;
        if (!mp_.cellIsObstacle(v.pnt_.x_, v.pnt_.y_)) {
            flagAdded = extend(v);
        }
        ++cnt;
        if (flagAdded) {
            rewireNeighbors(v.selfItr_);
            reduceInconsistency();
            if(achievedStartState(v)){
                vBotIsAdded_ = true;
                vBot_ = v.selfItr_;
                updatePathNeeded = true;
                if(!guidePathDefined){
                    updatePath();
                    resetTreeFillGuidePath();
                    guidePathDefined = true;
                }
            }
        }
        if(updatePathNeeded){
            updatePath();
            updatePathNeeded = false;
        }
        diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
    }
//    cerr << "Elapsed time:" << diffTime/1000.0 << "ms" << endl;
    return vBotIsAdded_;
}

bool updateRobot() {
    return false;
}


void RRTX::verifyQueue(Node *vItr) {
    qRewiring_.push({{min(vItr->g_, vItr->lmc_), vItr->g_}, vItr});
}

void RRTX::saturate(Node &v, Node *&vNearest) {
    double cost = distanceFunction(v.pnt_, vNearest->pnt_);
    v.pnt_.x_ = vNearest->pnt_.x_ + (v.pnt_.x_ - vNearest->pnt_.x_) * DELTA / cost;
    v.pnt_.y_ = vNearest->pnt_.y_ + (v.pnt_.y_ - vNearest->pnt_.y_) * DELTA / cost;
}

Node *RRTX::nearest(Node &v) {
    double mnCost = INF+10;
    Node *ans = NULL;
    for (auto &it: nodes_) {
        if (it.parent_ == NULL)
            continue;
        if (distanceFunction(v.pnt_, it.pnt_) < mnCost) {
            ans = it.selfItr_;
            mnCost = distanceFunction(v.pnt_, it.pnt_);
        }
    }
    return ans;
}

void RRTX::moveRobot(double shiftDeltaX, double shiftDeltaY, double newTheta) {
    shiftTree(-shiftDeltaX, -shiftDeltaY);
    shiftGoal(-shiftDeltaX, -shiftDeltaY);
    if(guidePathDefined){
        for(auto &it:guidePath){
            it.x_ -= shiftDeltaX;
            it.y_ -= shiftDeltaY;
        }
    }
    startPoint_.theta_=newTheta;
    updatePathNeeded = true;
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        if (isOut(it->pnt_)) {
            auto it1 = it;
            ++it;
            deleteNode(it1);
            continue;
        }
        ++it;
    }
    if (vBotIsAdded_) {
        if (hypot(vBot_->pnt_.x_ - startPoint_.x_, vBot_->pnt_.y_ - startPoint_.y_) > EPS_DOUBLE) {
            vBotIsAdded_ = false;
            updatePathNeeded = true;
            vBot_ = NULL;
        }
    }
    propagateDescendants();
    reduceInconsistency();
}

void RRTX::shiftGoal(double shiftDeltaX, double shiftDeltaY) {
    goal_.x_ += shiftDeltaX;
    goal_.y_ += shiftDeltaY;
}

void RRTX::drawTree(string fileName) {
    ofstream out;
    out.open(fileName.c_str());
    for (auto &it:nodes_) {
        if (it.parent_ != NULL){
            vector<Point> v;
            steerTrajectory(it.pnt_, it.parent_->pnt_, v, 1);
            for(int i=0; i+1<v.size(); i+=1){
                out<<v[i].x_<<" "<<v[i].y_<<" "<<v[i+1].x_<<" "<<v[i+1].y_<<endl;
            }
        }
        else{
            out << it.pnt_.x_<<" "<<it.pnt_.y_<<endl;
        }
    }
    out.close();
}

void RRTX::updateMap(Map &mp) {
    mp_ = mp;
    checkForDisappearedObstacles();
    checkForAppearedObstacles();
    propagateDescendants();
    reduceInconsistency();
}

void RRTX::checkForDisappearedObstacles() {
    for(auto &it:nodes_){
        for(auto it1=it.NPlusNotConnected0_.begin(); it1!=it.NPlusNotConnected0_.end();){
            if(steer(it.pnt_, (*it1)->pnt_)){
                it.NPlus0_.insert((*it1)->selfItr_);
                (*it1)->NMinusR_.insert(it.selfItr_);
                if(distanceBySteerFunction(it.pnt_, (*it1)->pnt_)+(*it1)->lmc_<it.lmc_){
                    it.lmc_ = distanceBySteerFunction(it.pnt_, (*it1)->pnt_)+(*it1)->lmc_;
                    if(it.parent_!=NULL)
                        it.parent_->children_.erase(it.selfItr_);
                    it.parent_ = (*it1)->selfItr_;
                    (*it1)->children_.insert(it.selfItr_);
                    verifyQueue(it.selfItr_);
                }
                (*it1)->NMinusNotConnectedR_.erase(it.selfItr_);
                it1 = it.NPlusNotConnected0_.erase(it1);
            }
            else
                ++it1;
        }

        for(auto it1=it.NPlusNotConnectedR_.begin(); it1!=it.NPlusNotConnectedR_.end();){
            if(steer(it.pnt_, (*it1)->pnt_)){
                it.NPlusR_.insert((*it1)->selfItr_);
                (*it1)->NMinus0_.insert(it.selfItr_);
                if(distanceBySteerFunction(it.pnt_, (*it1)->pnt_)+(*it1)->lmc_<it.lmc_){
                    it.lmc_ = distanceBySteerFunction(it.pnt_, (*it1)->pnt_)+(*it1)->lmc_;
                    if(it.parent_!=NULL)
                        it.parent_->children_.erase(it.selfItr_);
                    it.parent_ =(*it1)->selfItr_;
                    (*it1)->children_.insert(it.selfItr_);
                    verifyQueue(it.selfItr_);
                }
                (*it1)->NMinusNotConnected0_.erase(it.selfItr_);
                it1 = it.NPlusNotConnectedR_.erase(it1);
            }
            else
                ++it1;
        }
    }
}

void RRTX::updateMapFromFile(string fileName) {
    ifstream in;
    in.open(fileName.c_str());
    int w, h, x;
    in>>h>>w;
    Map mp(h, w);
    for(int i=0; i<h; ++i){
        for(int j=0; j<w; ++j){
            in>>x;
            mp.setCell(i, j, x);
        }
    }
    updateMap(mp);
}

void RRTX::moveObstacles(string fileName, int shift) {
    int dx[8] = {1, 1, 1, -1, -1, -1, 0, 0},
    dy[8] = {0, 1, -1, 0, 1, -1, 1, -1};
    ifstream in;
    in.open(fileName.c_str());
    int w, h, x;
    in>>h>>w;
    Map mp2(h, w);
    for(int i=0; i<h; ++i){
        for(int j=0; j<w; ++j){
            in>>x;
            mp2.setCell(i, j, x);
        }
    }
    in.close();
    Map mp1;
    mp1 = mp_;
    for(int i=0; i<mp_.getHeight(); ++i)
        for(int j=0; j<mp_.getWidth(); ++j)
            mp1.setCell(i, j, 0);
    int tmp = 0;
    for(int x=0; x<mp2.getWidth(); ++x){
        for(int y=0; y<mp2.getHeight(); ++y){
            if(mp2.cellIsObstacle(x, y)){
                tmp = (y/20)%8;
                int xx = x+shift*dx[tmp], yy = y+shift*dy[tmp];
                if(mp2.isIn(xx, yy))
                    mp1.setCellXY(xx, yy, 1);
            }
        }
    }
    mp_ = mp1;
    checkForDisappearedObstacles();
//    reduceInconsistency();
    checkForAppearedObstacles();
    propagateDescendants();
    reduceInconsistency();
}

void RRTX::drawMap(string fileName) {
    ofstream out;
    out.open(fileName.c_str());
    out<<mp_.getHeight()<<" "<<mp_.getWidth()<<endl;
    for(int i=0; i<mp_.getHeight(); ++i){
        for(int j=0; j<mp_.getWidth(); ++j){
            out<<mp_.getValue(i, j)<<" ";
        }
        out<<endl;
    }
    out.close();
}

bool RRTX::achievedStartState(Node v) {
    // Assuming both angles in [0, 2*PI] format.
    double diffAngle = abs(v.pnt_.theta_ - startPoint_.theta_);
    diffAngle = min(diffAngle, 2*M_PI - diffAngle);
    return hypot(v.pnt_.x_ - startPoint_.x_, v.pnt_.y_ - startPoint_.y_) < EPS_GOAL
            && diffAngle <= EPS_ORIENTATION_ANGLE;
}

double RRTX::distanceBySteerFunction(Point &v1, Point &v2) {
    if(!steer(v1, v2))
        return INF;
    Dubins d(MIN_RADIUS, 0);
    return d.distance(&v1, &v2);
}

void RRTX::steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory, double step) {
    Dubins d(MIN_RADIUS, 0);
    auto path=d.dubins(&v1, &v2);
    vector<Point> v;
    int timeSteps = d.distance(&v1, &v2)/step+2;
    for(int i=1; i<=timeSteps; ++i){
        Point p;
        d.interpolate(&v1, path, (double)i/timeSteps, &p);
        trajectory.push_back(p);
    }
}

void RRTX::updatePath() {
    updatePathNeeded = false;
    finalPath.clear();
    if (vBotIsAdded_) {
        auto it = vBot_;
        vector<Point> path;
        while (it->parent_ != it->selfItr_) {
            path.push_back(Point(it->pnt_.x_, it->pnt_.y_, it->pnt_.theta_));
            it = it->parent_;
        }
        path.push_back(Point(it->pnt_.x_, it->pnt_.y_, it->pnt_.theta_));
        auto startTime = high_resolution_clock::now();
        PathSimplifier ps(mp_);
        ps.simplifyMax(path);
        double diffTime1 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
        for(int i=0; i+1<path.size(); ++i){
            auto it = path[i], it1 = path[i+1];
            vector<Point> v;
            steerTrajectory(it, it1, v, 1);
            for(auto it:v){
//                assert(mp_.cellIsObstacle(it.x_, it.y_)==0);
                finalPath.push_back(it);
            }
        }
    }
    return;
}


void RRTX::resetTreeFillGuidePath() {
    guidePath.clear();
    for(auto it:finalPath){
        guidePath.push_back(it);
    }
    counterAddPath = 1;
    finalPath.clear();
    initializeTree();
}

bool RRTX::checkDubinsPath(Dubins &d, DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy, double t1, double t2) {
    if(abs(int(stx+0.5)-int(ndx+0.5)) + abs(int(sty+0.5)-int(ndy+0.5))<=1 && t2-t1<1-EPS_DOUBLE){
        set<int> vx = {int(stx+0.5)}, vy = {int(sty+0.5)};
        if(abs(stx-(int(stx)+0.5))<=EPS_DOUBLE){
            vx.insert(ceil(stx));vx.insert(floor(stx));
        }
        if(abs(ndx-(int(ndx)+0.5))<=EPS_DOUBLE){
            vx.insert(ceil(ndx));vx.insert(floor(ndx));
        }
        if(abs(sty-(int(sty)+0.5))<=EPS_DOUBLE){
            vy.insert(ceil(sty));vy.insert(floor(sty));
        }
        if(abs(ndy-(int(ndy)+0.5))<=EPS_DOUBLE){
            vy.insert(ceil(ndy));vy.insert(floor(ndy));
        }
        for(auto itx:vx){
            for(auto ity:vy){
                if(mp_.cellIsObstacle(itx, ity)){
                    return false;
                }
            }
        }
        return true;
    }
    //    if(abs(int(stx+0.5)-int(ndx+0.5)) + abs(int(sty+0.5)-int(ndy+0.5))<=1 && t2-t1<1-EPS_DOUBLE){ // assure the start and end points are not in the same cell.
//        return !mp_.cellIsObstacle(stx, sty) && !mp_.cellIsObstacle(ndx, ndy);
//        bool ans =  !mp_.cellIsObstacle(stx, sty);
//        double tmpx = stx, tmpy = sty;
//        if(stx < int(stx+0.5))
//            tmpx = stx-0.5;
//        else if(stx > int(stx+0.5))
//            tmpx = stx+0.5;
//        if(sty < int(sty+0.5))
//            tmpy = sty-0.5;
//        else if(sty > int(sty+0.5))
//            tmpy = sty+0.5;
//        ans = ans && !mp_.cellIsObstacle(tmpx, sty) && !mp_.cellIsObstacle(stx, tmpy) && !mp_.cellIsObstacle(tmpx, tmpy);
//        if((int(stx+0.5) != int(ndx+0.5)) || (int(sty+0.5) != int(ndy+0.5))){
//            ans = ans && !mp_.cellIsObstacle(ndx, ndy);
//
//            tmpx = ndx, tmpy = ndy;
//            if(ndx < int(ndx+0.5))
//                tmpx = ndx-0.5;
//            else if(ndx > int(ndx+0.5))
//                tmpx = ndx+0.5;
//            if(ndy < int(ndy+0.5))
//                tmpy = ndy-0.5;
//            else if(ndy > int(ndy+0.5))
//                tmpy = ndy+0.5;
//            ans = ans && !mp_.cellIsObstacle(tmpx, ndy) && !mp_.cellIsObstacle(ndx, tmpy) && !mp_.cellIsObstacle(tmpx, tmpy);
//        }
//        return ans;
//    }
    Point md;
    double mdT = (t1+t2)/2.0;
    d.interpolate(p0, dp, mdT, &md);
    return checkDubinsPath(d, dp, p0, stx, sty, md.x_, md.y_, t1, mdT) &&
        checkDubinsPath(d, dp, p0, md.x_, md.y_, ndx, ndy, mdT, t2);
}

//bool RRTX::checkPartOfLeftCircle(double cntrX, double cntrY, double r, double stx, double sty, double ndx, double ndy) {
//    double x1 = cntrX + r;
//    double y1 = cntrY + r;
//    double x2 = cntrX - r;
//    double y2 = cntrY - r;
//    if(stx < x1 && stx > )
//}


#endif //RRTX_RRTX_CPP
