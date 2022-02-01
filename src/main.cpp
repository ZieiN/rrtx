#include <bits/stdc++.h>
#include "../include/rrtx.h"
#include "../include/constants.h"

using namespace std;
using namespace std::chrono;

// coefficient decreases by the equation c_i =  (SIGMA)^i where SIGMA < 1
const double SIGMA = 0.9;
const double STEP = 0.1; // length of each step
const int NUMBER_POINTS_TO_COMPARE = 200; //  total compare length = NUMBER_POINTS_TO_COMPARE * STEP

double measureDistraction(vector<Point> &oldPath, vector<Point> &path, ofstream &out){
    double ans = 0;
    int sz = min(oldPath.size(), path.size());
//    cout<<"sz="<<sz<<endl;
    for(int i=0; i<sz; ++i){
        double c = pow(SIGMA, i*STEP);
        ans +=  c * hypot(oldPath[i].x_-path[i].x_, oldPath[i].y_-path[i].y_);
        out<<path[i].x_<<" "<<path[i].y_<<" "<<oldPath[i].x_<<" "<<oldPath[i].y_<<" "<<path[i].x_<<" "<<path[i].y_<<endl;
//        if(i>0){
//            cout<<hypot(oldPath[i].x_-oldPath[i-1].x_, oldPath[i].y_-oldPath[i-1].y_)<<" "<<
//            hypot(path[i].x_-path[i-1].x_, path[i].y_-path[i-1].y_)<<endl;
//        }
//        cout<<hypot(oldPath[i].x_-path[i].x_, oldPath[i].y_-path[i].y_)<<endl;
//        cout<<c<<" "<<path[i].x_<<","<<path[i].y_<<" "<<oldPath[i].x_<<" "<<oldPath[i].y_<<endl;
    }
    for(int i=sz; i<path.size(); ++i){
        out<<path[i].x_<<" "<<path[i].y_<<endl;
    }
    if(oldPath.size()==0)
        return -1;
//    cout<<ans/(double)sz<<endl;
    return ans/(double)(sz);
}
vector<Point> oldPath;
Point moveRobot(ofstream &out, RRTX rrtx, string fileName){

    if(rrtx.vBotIsAdded_){
        if(rrtx.updatePathNeeded){
            assert(0);
            rrtx.updatePath(1);
        }
        vector<Point> path;
        path.push_back(rrtx.finalPath[0]);
        for(int i=0; i+1<rrtx.finalPath.size(); ++i){
            path.push_back(rrtx.finalPath[i+1]);
//            rrtx.steerTrajectory(rrtx.finalPath[i], rrtx.finalPath[i+1], path, STEP);
        }
        int tmp = 0;
        double dist = 0;
        while(dist<1 && tmp+1<path.size() && !rrtx.mp_.cellIsObstacle(path[tmp+1].x_, path[tmp+1].y_)){
            ++tmp;
            dist += hypot(path[tmp].x_-path[tmp-1].x_,
                          path[tmp].y_-path[tmp-1].y_);
        }
        ofstream out1;
        out1.open(fileName.c_str());
        double dis = measureDistraction(oldPath, path, out1);
        out<< dis <<endl;
        cout<<setprecision(5)<<dis<<endl;
//        if(dis>0.4){
//            exit(0);
//        }
        oldPath.clear();
        for(int i=tmp; i<min(tmp+NUMBER_POINTS_TO_COMPARE, (int)path.size()); ++i){
            oldPath.push_back(Point(path[i].x_-(path[tmp].x_-rrtx.startPoint_.x_),path[i].y_-(path[tmp].y_-rrtx.startPoint_.y_), path[i].theta_));
        }
        out1.close();
        return Point(path[tmp].x_-rrtx.startPoint_.x_, path[tmp].y_-rrtx.startPoint_.y_, path[tmp].theta_);
        // cout<<"Shifting..."<<endl;
        // cout<<path[tmp].x_<<" "<<path[tmp].y_<<endl;
    }
    else{
        ofstream out1;
        out1.open(fileName.c_str());
        out1.close();
        out<<-1<<endl;
        return Point(0, 0, rrtx.startPoint_.theta_);
    }
}


Map addDynamicObstacles(RRTX &rrtx, string fileName, int shift, double shX, double shY) {
//    int dx[8] = {1, 1, 1, -1, -1, -1, 0, 0},
//            dy[8] = {0, 1, -1, 0, 1, -1, 1, -1};
        int dx[4] = {0, 0, 1, -1}, dy[4] = {1, -1, 0, 0};

    srand(1);
    vector<pair<int, int>> v;
    vector<int> sz;
    for(int i=0; i<10000; ++i){
        v.emplace_back(rand()%rrtx.mp_.getWidth(), rand()%rrtx.mp_.getHeight());
        sz.emplace_back(rand()%20+2);
    }
    ifstream in;
    in.open(fileName.c_str());
    int w, h, x;
    in>>h>>w;
    Map mp2(h, w), mp(h, w);
    for(int i=0; i<h; ++i){
        for(int j=0; j<w; ++j){
            in>>x;
            mp2.setCell(i, j, x);
        }
    }
    in.close();
    for(int i=0; i<200; ++i){
        int id = i%8;
        int x = v[i].first + dx[id]*shift, y = v[i].second+dy[id]*shift;
        bool fl = true;
        for(int j=-sz[i]/2; j<sz[i]/2; ++j){
            for(int k=-sz[i]/2; k<sz[i]/2; ++k){
//                if(x+j-shX == rrtx.startPoint_.x_ && y+k-shY==rrtx.startPoint_.y_){
               if((abs(x+j-shX - rrtx.goal_.x_)<=2 && abs(y+k-shY-rrtx.goal_.y_)<=2)){
                   fl = false;
                   break;
                }
            }
        }
        if(fl) {
            for (int j = -sz[i] / 2; j < sz[i] / 2; ++j) {
                for (int k = -sz[i] / 2; k < sz[i] / 2; ++k) {
                    if (rrtx.mp_.isIn(x + j, y + k))
                        mp2.setCellXY(x + j, y + k, 1);
                }
            }
        }
    }
    int shiftI = -shY, shiftJ = shX;
    for(int i=0; i<mp2.getHeight(); ++i){
        for(int j=0; j<mp2.getWidth(); ++j){
            if(mp2.isIn(i+shiftI, j+shiftJ) && mp2.cellIsObstacleIJ(i+shiftI, j+shiftJ)){
                mp.setCell(i, j, 1);
            }
            else{
                mp.setCell(i, j, 0);
            }
        }
    }
//    for(int y=0; y<rrtx.mp_.getHeight(); ++y){
//        for(int x=0; x<rrtx.mp_.getWidth(); ++x){
//            double xx = x+shX;
//            double yy = y+shY;
//            bool ans = true;
//            for(double k=-0.5; k<=0.5; ++k){
//                for(double l=-0.5; l<=0.5; ++l){
//                    ans = (ans && mp2.isInXY(xx+k, yy+l) && mp2.cellIsObstacle(xx+k, yy+l));
//                }
//            }
//            if(ans)
////            if(mp2.isIn(x, y) && mp2.cellIsObstacle(x, y))
//                rrtx.mp_.setCellXY(x, y, 1);
//            else
//                rrtx.mp_.setCellXY(x, y, 0);
//        }
//    }
//    mp_ = mp2
//    rrtx.checkForDisappearedObstacles();
//    rrtx.checkForAppearedObstacles();
//    rrtx.propagateDescendants();
//    rrtx.reduceInconsistency();
    return mp;
}


int main() {
    for(int imageNum = 2; imageNum<=3; ++imageNum) {
        string imageName = to_string(imageNum);
        string inputsFile = "../input/map" + imageName + "/inputs.txt";
        ifstream in;
        string timesFile = "../output/map" + imageName + "/Dynamics/times.txt";
        ofstream outTime, outShifts, outDistraction;
        in.open(inputsFile.c_str());
        outTime.open(timesFile.c_str());
        double stx, sty, sto, ndx, ndy, ndo;
        int cnt = 0;
        while (in >> stx >> sty >> sto >> ndx >> ndy >> ndo) {
          if(cnt<10){++cnt;continue;}
            Point start = Point(stx, sty, sto);
            Point goal = Point(ndx, ndy, ndo);
            RRTX rrtx(start, goal);
            rrtx.updateMapFromFile("../input/map" + imageName + "/image.txt");
            string shiftFile = "../output/map" + imageName + "/Dynamics/shifts-"+ to_string(cnt)+".txt";
            string distractionFile = "../output/map" + imageName + "/Dynamics/distraction-"+ to_string(cnt)+".txt";
            outShifts.open(shiftFile.c_str());
            outDistraction.open(distractionFile.c_str());
            double shX = 0, shY = 0, shX1 = 0, shY1 = 0;
            for (int i = 0; i < 450 && rrtx.distanceFunction(rrtx.goal_, rrtx.startPoint_)>EPS_GOAL; ++i) {
                cerr<<imageNum<<" "<<cnt<<" "<<i<<"::";
                auto shift = moveRobot(outDistraction, rrtx, "../output/map" + imageName + "/Dynamics/outSol" + to_string(cnt) + "-" + to_string(i) +
                                                             ".txt");
//                cout<<fixed<<setprecision(3)<<shift.x_<<" "<<shift.y_<<endl;
                int shiftMapX, shiftMapY;
                if(rrtx.startPoint_.x_ > 299){
                    shiftMapX = ceil(shift.x_);
                }
                else{
                    shiftMapX = floor(shift.x_);
                }
                if(rrtx.startPoint_.y_ > 300){
                    shiftMapY = ceil(shift.y_);
                }
                else{
                    shiftMapY = floor(shift.y_);
                }
                shX += shiftMapX; shY += shiftMapY;
                shX1 += shift.x_-shiftMapX; shY1 += shift.y_-shiftMapY;
                auto mp = addDynamicObstacles(rrtx, "../input/map" + imageName + "/image.txt", i, shX, shY);
                auto startTime = high_resolution_clock::now();
                rrtx.startPoint_.x_ += shift.x_-shiftMapX; rrtx.startPoint_.y_ += shift.y_-shiftMapY;
                for(auto &it:oldPath){
                    it.x_+=shift.x_-shiftMapX;
                    it.y_+=shift.y_-shiftMapY;
                }
                rrtx.moveRobot(shiftMapX, shiftMapY, shift.theta_);
                rrtx.updateMap(mp);
                outShifts<<shX<<" "<<shY<<" "<<shX1<<" "<<shY1<<endl;
                double diffTime1 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                startTime = high_resolution_clock::now();
                rrtx.search();
                auto diffTime2 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                outTime <<diffTime1<<" "<< diffTime2 << endl;
                // cout<<fixed<<setprecision(3) <<diffTime1/1000.0<<" "<< diffTime2/1000.0 << " n_nodes=" <<rrtx.nodes_.size()<<",n_edges="<<rrtx.edges.size()<< endl;
                rrtx.drawTree("../output/map" + imageName + "/Dynamics/outTree" + to_string(cnt) + "-" + to_string(i) +
                              ".txt");
//                rrtx.drawSolution(
//                        "../output/map" + imageName + "/Dynamics/outSol" + to_string(cnt) + "-" + to_string(i) +
//                        ".txt");
                rrtx.drawMap("../output/map" + imageName + "/Dynamics/outMap" + to_string(cnt) + "-" + to_string(i) +
                             ".txt");

            }
            outShifts.close();
            outDistraction.close();
           break;
            ++cnt;
        }
       break;
    }
    return 0;
}
