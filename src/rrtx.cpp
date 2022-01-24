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
    edges.clear();
    dubins_ = Dubins(MIN_RADIUS, 0);
    for(int i=0; i<MX_WIDTH; ++i)
        passedEdges[i].clear();
    shiftX_ = shiftY_ = 0;
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
    shiftX_ = shiftY_ = 0;
    dubins_ = Dubins(MIN_RADIUS, 0);
    initializeTree();
}

//bool operator<(pair<pair<double, double>, Node *> a, pair<pair<double, double>, Node *> b) {
//    return a.first.first > b.first.first || (a.first.first == b.first.first && a.first.second > b.first.second);
//}
bool operator < (const std::_List_iterator<std::pair<std::pair<Node*, Node*>, pair<DubinsPath, std::pair<bool, int> > > > a, const std::_List_iterator<std::pair<std::pair<Node*, Node*>, pair<DubinsPath, std::pair<bool, int> > > > b){
    if(a->first.first == b->first.first)return a->first.second < b->first.second;
    return a->first.first < b->first.first;
}
bool operator < (const DubinsPath dp1, const DubinsPath dp2){
    return dp1.type_ < dp2.type_;
}
Point RRTX::randomNode() {
    if(guidePathDefined && counterAddPath < guidePath.size()){
        counterAddPath += 10;
        return guidePath[(int)(guidePath.size())-1-counterAddPath-10];
    }
    random_device rd;
//    static int cnt = 0;
//    mt19937 mt(cnt++);
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
//         x = int(x*1000)/1000.0;
//         y = int(y*1000)/1000.0;
//         ang = int(ang*1000)/1000.0;
         return Point(x, y, ang);
     }
     else {
         uniform_real_distribution<double> randHeight(0, MX_HEIGHT);
         uniform_real_distribution<double> randWidth(0, MX_WIDTH);
         uniform_real_distribution<double> randOrientation(0, 2 * M_PI);
         uniform_real_distribution<double> randForStartPoint(0, 1);
         double x = randWidth(mt);
         double y = randHeight(mt);
         double theta = randOrientation(mt);
//         x = int(x*1000)/1000.0;
//         y = int(y*1000)/1000.0;
//         theta = int(theta*1000)/1000.0;
         return Point(x, y, theta);
     }
}

void RRTX::shrinkingBallRadius() {
    radius_ = min(GAMMA * pow(log(nodes_.size() + 1) / (1.0 + nodes_.size()), 1.0 / DIM), DELTA);
}

bool RRTX::extend(Node &v) {
    list<pair<Node *, DubinsPath>> vNear = near(v);
    findParent(v, vNear);
    if (v.parent_ == NULL) {
        return false;
    }
    auto it = nodes_.insert(nodes_.end(), v);
    Node *vItr = &(*it);
    vItr->selfItr_ = v.selfItr_ = vItr;
    assert(&(*vItr) == vItr->selfItr_);
    if(deletedNodes_.find(vItr->selfItr_)!=deletedNodes_.end()){
        deletedNodes_.erase(vItr->selfItr_);
    }
    v.parent_->children_.insert(vItr);
    for (auto &it: vNear) {

        assert (checkDubinsPath(it.second, vItr->pnt_, it.first->pnt_));
        addEdge(vItr, {it.first, it.second}, 0);

        auto steerResult = steer(it.first->pnt_, vItr->pnt_);
        if(steerResult.first) {
            addEdge(it.first, {vItr, steerResult.second}, 1);
        }
    }
    return true;
}

list<pair<Node *, DubinsPath>> RRTX::near(Node &v) {
    list<pair<Node *, DubinsPath>> ans;
    for (auto &it: nodes_) {
        /// because I want to add the orphans to 'near'-set so they can be rewired.
        // if(it.parent_ == NULL)
        //     continue;
        if (distanceFunction(v.pnt_, it.pnt_) <= radius_) {
            auto steerResult = steer(v.pnt_, it.pnt_);
            if(steerResult.first) {
                ans.push_back({it.selfItr_, steerResult.second});
            }
        }
    }
    return ans;
}

void RRTX::findParent(Node &v, list<pair<Node *, DubinsPath>> &vNear) {
    // we prune the neighbor set here, in addition to find the parent.
    if(vNear.empty()) {
        return;
    }
    vector<pair<double, pair<Node *, DubinsPath>>> vectorNeighbors;
    for (auto &it: vNear) {
        auto costTransition = dubins_.distance(it.second);
        vectorNeighbors.emplace_back(costTransition+it.first->lmc_, it);
        if (costTransition + it.first->lmc_ < v.lmc_) {
            v.parent_ = it.first->selfItr_;
            v.lmc_ = costTransition + it.first->lmc_;
            v.pathToParent_ = it.second;
        }
    }
    sort(vectorNeighbors.begin(), vectorNeighbors.end());
    if(vNear.size()>MAX_NUM_NEIGHNORS){
        vNear.clear();
        for(int i=0; i<MAX_NUM_NEIGHNORS; ++i){
            vNear.push_back(vectorNeighbors[i].second);
        }
    }
}

double RRTX::distanceFunction(Point &v1, Point &v2) {
    return hypot(v1.x_ - v2.x_, v1.y_ - v2.y_);
}

pair<bool, DubinsPath> RRTX::steer(Point v1, Point &v2) {
    auto path=dubins_.dubins(&v1, &v2);
//    cout<<fixed<<setprecision(3)<<v1.x_+shiftX_<<" "<<v1.y_+shiftY_<<" "<<v2.x_+shiftX_<<" "<<v2.y_+shiftY_<<" "<<v1.theta_<<" "<<v2.theta_<<endl;
//    for(double i=0; i<=1; i+=0.05){
//        Point tmp;
//        dubins_.interpolate(&v1, path, i, &tmp);
//        cout<<fixed<<setprecision(3)<<tmp.x_<<" "<<tmp.y_<<" ";
//    }cout<<fixed<<setprecision(3)<<endl;
//    cout<<fixed<<setprecision(3)<<hypot(v1.x_-v2.x_, v1.y_-v2.y_)<<" "<<path.length()<<" "<<path.length_[0]<<" "<<path.length_[1]<<" "<<path.length_[2]<<endl;
//    for(int i=0; i<3; ++i)cout<<fixed<<setprecision(3)<<path.type_[i]<<" ";cout<<fixed<<setprecision(3)<<endl;
//    assert(hypot(v1.x_-v2.x_, v1.y_-v2.y_)<=path.length());
    bool flag = checkDubinsPath(path, &v1, v1.x_, v1.y_, v2.x_, v2.y_, 0, 1);
    return {flag, path};
}

/*
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
//    cout<<fixed<<setprecision(3)<<"SUM = "<<sm<<endl;
}
*/

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
            it1.first->g_ = INF;
            verifyQueue(it1.first);
        }
        for (auto it1: it->NPlus0_) {
            it1.first->g_ = INF;
            verifyQueue(it1.first);
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
        if (it->first->selfItr_ == vItr->parent_ || distanceFunction(it->first->pnt_, vItr->pnt_)<=radius_) {
            ++it;
            continue;
        }
        auto it1 = it;
        ++it;
        it1->first->NMinus0_.erase({vItr, it1->second});
        vItr->NPlusR_.erase(it1);
    }
}

// Search the nodes around me (vItr) which I can connect through them, and try to improve my lmc through them.
void RRTX::updateLMC(Node *vItr) {
    cullNeighbors(vItr);
    for (auto &it: vItr->NPlus0_) {
        if (it.first->parent_ == vItr) {
            continue;
        }
        if (/* steer(vItr->pnt_, it->pnt_) &&*/
        vItr->lmc_ > dubins_.distance(it.second) + it.first->lmc_ && orphans_.find(it.first)==orphans_.end()) {
            vItr->lmc_ = distanceBySteerFunction(vItr->pnt_, it.first->pnt_) + it.first->lmc_;
            if (vItr->parent_ != NULL)
                (vItr->parent_)->children_.erase(vItr->selfItr_);
            vItr->parent_ = it.first->selfItr_;
            vItr->pathToParent_ = it.second;
            (vItr->parent_)->children_.insert(vItr->selfItr_);
        }
    }
    for (auto &it: vItr->NPlusR_) {
        if (it.first->parent_ == vItr) {
            continue;
        }
        if (/*steer(vItr->pnt_, it->pnt_) && */
        vItr->lmc_ > dubins_.distance(it.second) + it.first->lmc_ && orphans_.find(it.first)==orphans_.end()) {
            vItr->lmc_ = distanceBySteerFunction(vItr->pnt_, it.first->pnt_) + it.first->lmc_;
            if (vItr->parent_ != NULL)
                (vItr->parent_)->children_.erase(vItr->selfItr_);
            vItr->parent_ = it.first->selfItr_;
            vItr->pathToParent_ = it.second;
            (vItr->parent_)->children_.insert(vItr->selfItr_);
        }
    }
}


// For all nodes in the queue, if their difference (g-lmc) is bigger than delta,
// then update the lmc for the best value and then assign it to g_.
void RRTX::reduceInconsistency() {
//    cout<<fixed<<setprecision(3)<<qRewiring_.size()<<endl;
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
            if (it.first->selfItr_ == vItr->parent_)
                continue;
            if (it.first->lmc_ > dubins_.distance(it.second) + (vItr->lmc_)) {
                it.first->lmc_ = dubins_.distance(it.second) + vItr->lmc_;
                if (it.first->parent_ != NULL)
                    (it.first->parent_)->children_.erase(it.first->selfItr_);
                it.first->parent_ = vItr->selfItr_;
                it.first->pathToParent_ = it.second;
                it.first->parent_->children_.insert(it.first->selfItr_);
                if (achievedStartState(*it.first)){
                    if(!vBotIsAdded_) {
                        vBotIsAdded_ = true;
                        updatePathNeeded = true;
                        vBot_ = it.first->selfItr_;
                    }
                    else if(vBot_->lmc_>it.first->lmc_){
                        vBotIsAdded_ = true;
                        updatePathNeeded = true;
                        vBot_ = it.first->selfItr_;
                    }
                }
                if (it.first->g_ - it.first->lmc_ > EPS) {
                    verifyQueue(it.first->selfItr_);
                }
            }
        }
        for (auto &it: vItr->NMinusR_) {
            if (it.first->selfItr_ == vItr->parent_)
                continue;
            if (it.first->lmc_ > dubins_.distance(it.second) + (vItr->lmc_)) {
                it.first->lmc_ = dubins_.distance(it.second) + vItr->lmc_;
                if (it.first->parent_ != NULL)
                    it.first->parent_->children_.erase(it.first->selfItr_);
                it.first->parent_ = vItr->selfItr_;
                it.first->pathToParent_ = it.second;
                it.first->parent_->children_.insert(it.first->selfItr_);
                if (achievedStartState(*it.first)) {
                    if(!vBotIsAdded_) {
                        vBotIsAdded_ = true;
                        updatePathNeeded = true;
                        vBot_ = it.first->selfItr_;
                    }
                    else if(vBot_->lmc_>it.first->lmc_){
                        vBotIsAdded_ = true;
                        updatePathNeeded = true;
                        vBot_ = it.first->selfItr_;
                    }
                }
                if (it.first->g_ - it.first->lmc_ > EPS) {
                    verifyQueue(it.first->selfItr_);
                }
            }
        }
    }
}
bool isOut(Point &p) {
    if (p.x_ < 0 || p.x_ >= MX_WIDTH || p.y_ < 0 || p.y_ >= MX_HEIGHT)
        return true;
    return false;
}

void RRTX::check() {

    for(auto it=edges.begin(); it!=edges.end(); ++it){
        auto &nod1 = (it)->first.first;
        auto &nod2 = (it)->first.second;
        assert(deletedNodes_.find(nod1)==deletedNodes_.end());
        assert(deletedNodes_.find(nod2)==deletedNodes_.end());
        auto n1 = nod1->pnt_;
        auto n2 = nod2->pnt_;
        set<pair<int, int>> setPoints;
        getLatticePointsOnThePath(it->second.first, &n1, n1.x_, n1.y_, n2.x_, n2.y_, 0, 1, setPoints);
        for(auto pnt:setPoints){
            assert(passedEdges.count(pnt.first+shiftX_));
            assert(passedEdges[pnt.first+shiftX_].count(pnt.second+shiftY_));
            assert(passedEdges[pnt.first+shiftX_][pnt.second+shiftY_].find(it) != passedEdges[pnt.first+shiftX_][pnt.second+shiftY_].end());
        }
    }

    for (auto &it: nodes_) {
        assert(deletedNodes_.find(&it)==deletedNodes_.end());
        for (auto &it1: it.children_) {
            assert(it1->parent_ == it.selfItr_);
            assert(deletedNodes_.find(it1)==deletedNodes_.end());
//            assert(it.NMinus0_.find(it1)!=it.NMinus0_.end() || it.NMinusR_.find(it1)!=it.NMinusR_.end());
        }
        for (auto &it1: it.NPlusR_) {
            assert(deletedNodes_.find(it1.first)==deletedNodes_.end());
            assert(it1.first->NMinus0_.find({it.selfItr_, it1.second}) != it1.first->NMinus0_.end());
        }
        for (auto &it1: it.NPlus0_) {
            assert(deletedNodes_.find(it1.first)==deletedNodes_.end());
            assert(it1.first->NMinusR_.find({it.selfItr_, it1.second}) != it1.first->NMinusR_.end());
        }
        for (auto &it1: it.NMinus0_) {
            assert(deletedNodes_.find(it1.first)==deletedNodes_.end());
            assert((it1.first->NPlusR_.find({it.selfItr_, it1.second})) != it1.first->NPlusR_.end());
        }
        for (auto &it1: it.NMinusR_) {
            assert(deletedNodes_.find(it1.first)==deletedNodes_.end());
            assert(it1.first->NPlus0_.find({it.selfItr_, it1.second}) != it1.first->NPlus0_.end());
        }
        if(it.parent_!=NULL) {
            if(it.parent_!=it.selfItr_) {
                assert(it.NPlusR_.find({it.parent_, it.pathToParent_}) != it.NPlusR_.end() ||
                       it.NPlus0_.find({it.parent_, it.pathToParent_}) != it.NPlus0_.end());
                assert(checkDubinsPath(it.pathToParent_, &it.pnt_, it.pnt_.x_, it.pnt_.y_, it.parent_->pnt_.x_, it.parent_->pnt_.y_, 0, 1));
            }
        }
        assert((&(it)) == (it.selfItr_));
    }
}

void RRTX::deleteNode(list<Node>::iterator &pntNode) {

    // The goal node shouldn't be called to be deleted. If this happens, ignore it.
    if(pntNode==nodes_.begin()){
        return;
    }

    for (auto &it: pntNode->NPlus0_) {
        it.first->NMinusR_.erase({pntNode->selfItr_, it.second});
        if (pntNode->parent_ == it.first->selfItr_) {
//            assert(it.first->children_.find(pntNode->selfItr_) != it.first->children_.end());
            it.first->children_.erase(pntNode->selfItr_);
        }
    }
    for (auto &it: pntNode->NPlusR_) {
        it.first->NMinus0_.erase({pntNode->selfItr_, it.second});
        if (pntNode->parent_ == it.first->selfItr_) {
//            assert(it.first->children_.find(pntNode->selfItr_) != it.first->children_.end());
            it.first->children_.erase(pntNode->selfItr_);
        }
    }
    for (auto &it: pntNode->NPlusNotConnected0_) {
//        assert(it.first->NMinusNotConnectedR_.find({pntNode->selfItr_, it.second})!=it.first->NMinusNotConnectedR_.end());
        it.first->NMinusNotConnectedR_.erase({pntNode->selfItr_, it.second});
    }
    for (auto &it: pntNode->NPlusNotConnectedR_) {
//        assert(it.first->NMinusNotConnected0_.find({pntNode->selfItr_, it.second})!=it.first->NMinusNotConnected0_.end());
        it.first->NMinusNotConnected0_.erase({pntNode->selfItr_, it.second});
    }
    for (auto &it: pntNode->NMinus0_) {
//        assert(it.first->NPlusR_.find({pntNode->selfItr_, it.second}) != it.first->NPlusR_.end());
        it.first->NPlusR_.erase({pntNode->selfItr_, it.second});
        if (it.first->parent_ == pntNode->selfItr_) {
            orphans_.insert(it.first);
            (it.first)->g_ = INF;
            it.first->parent_ = NULL;
        }
    }
    for (auto &it: pntNode->NMinusR_) {
//        assert(it.first->NPlus0_.find({pntNode->selfItr_, it.second}) != it.first->NPlus0_.end());
        it.first->NPlus0_.erase({pntNode->selfItr_, it.second});
        if (it.first->parent_ == pntNode->selfItr_) {
            orphans_.insert(it.first);
            (it.first)->g_ = INF;
            (it.first)->parent_ = NULL;
        }
    }
    for (auto &it: pntNode->NMinusNotConnected0_) {
//        assert(it.first->NPlusNotConnectedR_.find({pntNode->selfItr_, it.second})!=it.first->NPlusNotConnectedR_.end());
        it.first->NPlusNotConnectedR_.erase({pntNode->selfItr_, it.second});
    }
    for (auto &it: pntNode->NMinusNotConnectedR_) {
//        assert(it.first->NPlusNotConnected0_.find({pntNode->selfItr_, it.second})!=it.first->NPlusNotConnected0_.end());
        it.first->NPlusNotConnected0_.erase({pntNode->selfItr_, it.second});
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

void RRTX::shiftTree(int shiftDeltaX, int shiftDeltaY) {
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
        it->pnt_.x_ += shiftDeltaX;
        it->pnt_.y_ += shiftDeltaY;
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
    nodes_.begin()->NMinus0_.insert({nodes_.begin()->selfItr_, DubinsPath()});
    nodes_.begin()->NMinusR_.insert({nodes_.begin()->selfItr_, DubinsPath()});
    nodes_.begin()->NPlusR_.insert({nodes_.begin()->selfItr_, DubinsPath()});
    nodes_.begin()->NPlus0_.insert({nodes_.begin()->selfItr_, DubinsPath()});
    while (!qRewiring_.empty()) {
        qRewiring_.pop();
    }
    edges.clear();
    passedEdges.clear();
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
    while (!vBotIsAdded_ && diffTime < RUNTIME_LIMIT) {
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

void RRTX::moveRobot(int shiftDeltaX, int shiftDeltaY, double newTheta) {

    shiftX_ += shiftDeltaX;
    shiftY_ += shiftDeltaY;
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

    updateEdges();

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

void RRTX::shiftGoal(int shiftDeltaX, int shiftDeltaY) {
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
//    checkForDisappearedObstacles();
//    checkForAppearedObstacles();

//    auto startTime = high_resolution_clock::now();
    checkAllEdges();
//    double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count()/1000.0;
//    cout<<fixed<<setprecision(3)<<"Time to checkAllEdges:"<<diffTime<<endl;
//    startTime = high_resolution_clock::now();
    propagateDescendants();
//    diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count()/1000.0;
//    cout<<fixed<<setprecision(3)<<"Time to propagate:"<<diffTime<<endl;
//    startTime = high_resolution_clock::now();
    reduceInconsistency();
//    diffTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count()/1000.0;
//    cout<<fixed<<setprecision(3)<<"Time to reduceInconsistency:"<<diffTime<<endl;
//    check();
}
/*
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
*/
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
//    checkForDisappearedObstacles();
//    checkForAppearedObstacles();
    checkAllEdges();
    propagateDescendants();
    reduceInconsistency();
//    check();
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
    if(!(steer(v1, v2).first))
        return INF;
    return dubins_.distance(&v1, &v2);
}

void RRTX::steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory, double step) {
    auto path=dubins_.dubins(&v1, &v2);
    vector<Point> v;
    int timeSteps = dubins_.distance(path)/step+2;
    for(int i=1; i<=timeSteps; ++i){
        Point p;
        dubins_.interpolate(&v1, path, (double)i/timeSteps, &p);
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

bool RRTX::checkDubinsPath(const DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy, double t1, double t2) {
    if(abs(lround(stx)-lround(ndx)) + abs(lround(sty)-lround(ndy))<=1 && t2-t1<1-EPS_DOUBLE){
        set<int> vx = {static_cast<int>(lround(stx))}, vy = {static_cast<int>(lround(sty))};
        if(abs(abs(stx-lround(stx))-0.5)<=EPS_DOUBLE){
            vx.insert(ceil(stx));vx.insert(floor(stx));
        }
        if(abs(abs(ndx-lround(ndx))-0.5)<=EPS_DOUBLE){
            vx.insert(ceil(ndx));vx.insert(floor(ndx));
        }
        if(abs(abs(sty-lround(sty))-0.5)<=EPS_DOUBLE){
            vy.insert(ceil(sty));vy.insert(floor(sty));
        }
        if(abs(abs(ndy-lround(ndy))-0.5)<=EPS_DOUBLE){
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
    Point md;
    double mdT = (t1+t2)/2.0;
    dubins_.interpolate(p0, dp, mdT, &md);
    return checkDubinsPath(dp, p0, stx, sty, md.x_, md.y_, t1, mdT) &&
        checkDubinsPath(dp, p0, md.x_, md.y_, ndx, ndy, mdT, t2);
}

void
RRTX::getLatticePointsOnThePath(DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy,
                                double t1, double t2, set<pair<int, int>> &pointsSet) {
    if(abs(lround(stx)-lround(ndx)) + abs(lround(sty)-lround(ndy))<=1 && t2-t1<1-EPS_DOUBLE){
//        int vx = lround(stx), vy = lround(sty);
        set<int> vx, vy;
        vx.insert(lround(stx));
        vy.insert(lround(sty));
        if(abs(abs(stx-lround(stx))-0.5)<=EPS_DOUBLE){
            vx.insert(ceil(stx));vx.insert(floor(stx));
        }
        if(abs(abs(ndx-lround(ndx))-0.5)<=EPS_DOUBLE){
            vx.insert(ceil(ndx));vx.insert(floor(ndx));
        }
        if(abs(abs(sty-lround(sty))-0.5)<=EPS_DOUBLE){
            vy.insert(ceil(sty));vy.insert(floor(sty));
        }
        if(abs(abs(ndy-lround(ndy))-0.5)<=EPS_DOUBLE){
            vy.insert(ceil(ndy));vy.insert(floor(ndy));
        }
        for(auto itx:vx){
            for(auto ity:vy){
                pointsSet.insert({itx, ity});
            }
        }
        return;
    }
    Point md;
    double mdT = (t1+t2)/2.0;
    dubins_.interpolate(p0, dp, mdT, &md);
    getLatticePointsOnThePath(dp, p0, stx, sty, md.x_, md.y_, t1, mdT, pointsSet);
    getLatticePointsOnThePath(dp, p0, md.x_, md.y_, ndx, ndy, mdT, t2, pointsSet);
}

void RRTX::checkAllEdges() {
    for(auto &itx:passedEdges){
        for(auto &ity:itx.second){
            int x = itx.first-shiftX_, y = ity.first-shiftY_;
            if(mp_.cellIsObstacle(x, y)){
                for(auto id=ity.second.begin(); id!=ity.second.end(); ++id){
                    if((*id)->second.second.second == 1){
                        auto &nod1 = (*id)->first.first;
                        auto &nod2 = (*id)->first.second;
                        if((*id)->second.second.first){
//                            assert(nod1->NPlusR_.find({nod2, (*id)->second.first})!=nod1->NPlusR_.end());
                            nod1->NPlusR_.erase({nod2, (*id)->second.first});
                            nod1->NPlusNotConnectedR_.insert({nod2, (*id)->second.first});
//                            assert(nod2->NMinus0_.find({nod1, (*id)->second.first})!= nod2->NMinus0_.end());
                            nod2->NMinus0_.erase({nod1, (*id)->second.first});
                            nod2->NMinusNotConnected0_.insert({nod1, (*id)->second.first});
                        }
                        else{
//                            assert(nod1->NPlus0_.find({nod2, (*id)->second.first})!=nod1->NPlus0_.end());
                            nod1->NPlus0_.erase({nod2, (*id)->second.first});
                            nod1->NPlusNotConnected0_.insert({nod2, (*id)->second.first});
//                            assert(nod2->NMinusR_.find({nod1, (*id)->second.first})!=nod2->NMinusR_.end());
                            nod2->NMinusR_.erase({nod1, (*id)->second.first});
                            nod2->NMinusNotConnectedR_.insert({nod1, (*id)->second.first});
                        }

                        if (nod1->parent_ == nod2) {
                            orphans_.insert(nod1);
                            nod1->g_ = INF;
                            nod2->children_.erase(nod1);
                            nod1->parent_ = NULL;
                        }
                    }
                    (*id)->second.second.second = -1; // real invalid
                }
            }
        }
    }
    for(auto &it:edges){
        if(it.second.second.second == -1){
            it.second.second.second = 0;
        }
        else if(it.second.second.second==0){
            it.second.second.second = 1;
            auto nod1 = it.first.first;
            auto nod2 = it.first.second;
            if(it.second.second.first){
//                assert(nod1->NPlusNotConnectedR_.find({nod2, it.second.first})!=nod1->NPlusNotConnectedR_.end());
                nod1->NPlusNotConnectedR_.erase({nod2, it.second.first});
                nod1->NPlusR_.insert({nod2, it.second.first});
//                assert(nod2->NMinusNotConnected0_.find({nod1, it.second.first})!=nod2->NMinusNotConnected0_.end());
                nod2->NMinusNotConnected0_.erase({nod1, it.second.first});
                nod2->NMinus0_.insert({nod1, it.second.first});
            }
            else{
//                assert(nod1->NPlusNotConnected0_.find({nod2, it.second.first})!=nod1->NPlusNotConnected0_.end());
                nod1->NPlusNotConnected0_.erase({nod2, it.second.first});
                nod1->NPlus0_.insert({nod2, it.second.first});
//                assert(nod2->NMinusNotConnectedR_.find({nod1, it.second.first})!=nod2->NMinusNotConnectedR_.end());
                nod2->NMinusNotConnectedR_.erase({nod1, it.second.first});
                nod2->NMinusR_.insert({nod1, it.second.first});
            }

            if(distanceBySteerFunction(nod1->pnt_, nod2->pnt_)+nod2->lmc_ < nod1->lmc_){
                nod1->lmc_ = distanceBySteerFunction(nod1->pnt_, nod2->pnt_)+nod2->lmc_;
                if(nod1->parent_!=NULL)
                    nod1->parent_->children_.erase(nod1);
                nod1->parent_ = nod2;
                nod1->pathToParent_ = it.second.first;
                nod2->children_.insert(nod1);
                verifyQueue(nod1);
            }
        }
    }
}

void RRTX::updateEdges() {
    for(auto it=edges.begin(); it!=edges.end();){
        auto &nod1 = (it)->first.first;
        auto &nod2 = (it)->first.second;
        if(isOut(nod1->pnt_) ||
                isOut(nod2->pnt_)){
            auto n1 = nod1->pnt_, n2 = nod2->pnt_;
            auto it1 = it;
            set<pair<int, int>> setPoints;
            auto path = it1->second.first;
            getLatticePointsOnThePath(it1->second.first, &n1, n1.x_, n1.y_, n2.x_, n2.y_, 0, 1, setPoints);
            for(auto pnt:setPoints){
                assert(passedEdges.count(pnt.first+shiftX_));
                assert(passedEdges[pnt.first+shiftX_].count(pnt.second+shiftY_));
                assert(passedEdges[pnt.first+shiftX_][pnt.second+shiftY_].find(it1) != passedEdges[pnt.first+shiftX_][pnt.second+shiftY_].end());
                passedEdges[pnt.first+shiftX_][pnt.second+shiftY_].erase(it1);
            }
            ++it;
            edges.erase(it1);
            continue;
        }
        ++it;
    }
}
void RRTX::addEdge(Node *n1, pair<Node *, DubinsPath> n2, bool direction) {
    if(direction) {
        n1->NPlusR_.insert(n2);
        n2.first->NMinus0_.insert({n1, n2.second});
    }
    else{
        n1->NPlus0_.insert(n2);
        n2.first->NMinusR_.insert({n1, n2.second});
    }
    set<pair<int, int>> setPoints;
    edges.push_back({{n1, n2.first}, {n2.second, {direction, 1}}});
    auto id = --edges.end();
    getLatticePointsOnThePath(n2.second, &n1->pnt_, n1->pnt_.x_, n1->pnt_.y_,n2.first->pnt_.x_, n2.first->pnt_.y_, 0, 1, setPoints);
    for (auto it: setPoints) {
        passedEdges[it.first + shiftX_][it.second + shiftY_].insert(id);
    }
}

void RRTX::steerTrajectory(Point v1, Point &v2, vector<Point> &trajectory, DubinsPath &dubinsPath, double step) {
    vector<Point> v;
    int timeSteps = dubins_.distance(dubinsPath)/step+2;
    for(int i=1; i<=timeSteps; ++i){
        Point p;
        dubins_.interpolate(&v1, dubinsPath, (double)i/timeSteps, &p);
        trajectory.push_back(p);
    }
}

bool RRTX::checkDubinsPath(const DubinsPath &dubinsPath, Point &p0, Point &p1) {
    return checkDubinsPath(dubinsPath, &p0, p0.x_, p0.y_, p1.x_, p1.y_, 0, 1);
}


#endif //RRTX_RRTX_CPP
