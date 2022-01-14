#include <bits/stdc++.h>
#include "../include/rrtx.h"
#include "../include/constants.h"

using namespace std;
using namespace std::chrono;

Point moveRobot(RRTX rrtx){
    if(rrtx.vBotIsAdded_){
        if(rrtx.updatePathNeeded){
            rrtx.updatePath();
        }
        vector<Point> path;
        path.push_back(rrtx.finalPath[0]);
        for(int i=0; i+1<rrtx.finalPath.size(); ++i){
            rrtx.steerTrajectory(rrtx.finalPath[i], rrtx.finalPath[i+1], path, 1);
        }
        int tmp = 0, tmp1 = 0;
        double dist = 0;
        while(dist<1 && tmp1+1<path.size() && !rrtx.mp_.cellIsObstacle(path[tmp1+1].x_, path[tmp1+1].y_)){
            ++tmp1;
            dist += hypot(path[tmp1].x_-path[tmp1-1].x_,
                          path[tmp1].y_-path[tmp1-1].y_);
        }
//        for(int j = tmp1; j+1<path.size(); ++j){
//            auto x = path[j].x_, y = path[j].y_;
//            if(abs(x-round(x))<=EPS_DOUBLE && abs(y-round(y))<=EPS_DOUBLE){
//                tmp = j;
//                break;
//            }
//        }
        return Point(path[tmp1].x_-rrtx.startPoint_.x_, path[tmp1].y_-rrtx.startPoint_.y_, path[tmp1].theta_);
        cout<<"Shifting..."<<endl;
        cout<<path[tmp].x_<<" "<<path[tmp].y_<<endl;
    }
    else{
        return Point(0, 0, rrtx.startPoint_.theta_);
    }
}


Map addDynamicObstacles(RRTX &rrtx, string fileName, int shift, double shX, double shY) {
    int dx[8] = {1, 1, 1, -1, -1, -1, 0, 0},
            dy[8] = {0, 1, -1, 0, 1, -1, 1, -1};
    srand(0);
    vector<pair<int, int>> v;
    vector<int> sz;
    for(int i=0; i<100; ++i){
        v.emplace_back(rand()%rrtx.mp_.getWidth(), rand()%rrtx.mp_.getHeight());
        sz.emplace_back(rand()%20);
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
    for(int i=0; i<50; ++i){
        int id = i%8;
        int x = v[i].first + dx[id]*shift, y = v[i].second+dy[id]*shift;
        bool fl = true;
        for(int j=-sz[i]/2; j<sz[i]/2; ++j){
            for(int k=-sz[i]/2; k<sz[i]/2; ++k){
                if(x+j-shX == rrtx.startPoint_.x_ && y+k-shY==rrtx.startPoint_.y_){
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
    for(int imageNum = 1; imageNum<=3; ++imageNum) {
        string imageName = to_string(imageNum);
        string inputsFile = "../input/map" + imageName + "/inputs.txt";
        ifstream in;
        string timesFile = "../output/map" + imageName + "/Dynamics/times.txt";
        ofstream outTime, outShifts;
        in.open(inputsFile.c_str());
        outTime.open(timesFile.c_str());
        double stx, sty, sto, ndx, ndy, ndo;
        int cnt = 0;
        while (in >> stx >> sty >> sto >> ndx >> ndy >> ndo) {
            Point start = Point(stx, sty, sto);
            Point goal = Point(ndx, ndy, ndo);
            RRTX rrtx(start, goal);
            rrtx.updateMapFromFile("../input/map" + imageName + "/image.txt");
            string shiftFile = "../output/map" + imageName + "/Dynamics/shifts-"+ to_string(cnt)+".txt";
            outShifts.open(shiftFile.c_str());
            double shX = 0, shY = 0, shX1 = 0, shY1 = 0;
            for (int i = 0; i < 150 && rrtx.distanceFunction(rrtx.goal_, rrtx.startPoint_)>EPS_GOAL; ++i) {
//            rrtx.moveObstacles("../input/map"+imageName+"/image.txt", i);
                cerr<<imageNum<<" "<<cnt<<" "<<i<<"::";
                auto shift = moveRobot(rrtx);
                cout<<shift.x_<<" "<<shift.y_<<endl;
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
//                cout<<shiftMapX<<" "<<shiftMapY<<" "<<shift.x_-shiftMapX<<" "<<shift.y_-shiftMapY<<" "
//                <<rrtx.startPoint_.x_<<" "<<rrtx.startPoint_.y_<<endl;
                rrtx.moveRobot(shiftMapX, shiftMapY, shift.theta_);
                rrtx.updateMap(mp);
                outShifts<<shX<<" "<<shY<<" "<<shX1<<" "<<shY1<<endl;
                double diffTime1 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                startTime = high_resolution_clock::now();
                rrtx.search();
                auto diffTime2 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                outTime <<diffTime1<<" "<< diffTime2 << endl;
                cout <<diffTime1/1000.0<<" "<< diffTime2/1000.0 << endl;
                rrtx.drawTree("../output/map" + imageName + "/Dynamics/outTree" + to_string(cnt) + "-" + to_string(i) +
                              ".txt");
                rrtx.drawSolution(
                        "../output/map" + imageName + "/Dynamics/outSol" + to_string(cnt) + "-" + to_string(i) +
                        ".txt");
                rrtx.drawMap("../output/map" + imageName + "/Dynamics/outMap" + to_string(cnt) + "-" + to_string(i) +
                             ".txt");

            }
            outShifts.close();
//            break;
            ++cnt;
        }
//        break;
    }
    return 0;
}
