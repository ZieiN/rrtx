#include <bits/stdc++.h>
#include "../include/rrtx.h"
#include "../include/constants.h"

using namespace std;
using namespace std::chrono;

int main() {
    for(int imageNum = 1; imageNum<=1; ++imageNum) {
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
            double shX = 0, shY = 0;
            for (int i = 0; i < 150; ++i) {
//            rrtx.moveObstacles("../input/map"+imageName+"/image.txt", i);
                cerr<<imageNum<<" "<<cnt<<" "<<i<<"::";
                auto startTime = high_resolution_clock::now();

                if(rrtx.vBotIsAdded_){
                    if(rrtx.updatePathNeeded){
                        rrtx.updatePath();
                    }
                    int tmp = 0; double dist = 0;
                    while(dist<2){
                        ++tmp;
                        dist += hypot(rrtx.finalPath[tmp].x_-rrtx.finalPath[tmp-1].x_,
                                      rrtx.finalPath[tmp].y_-rrtx.finalPath[tmp-1].y_);
                    }
                    shX += rrtx.finalPath[tmp].x_-rrtx.finalPath[0].x_;
                    shY += rrtx.finalPath[tmp].y_-rrtx.finalPath[0].y_;
                    cout<<"Shifting..."<<endl;
                    cout<<shX<<" "<<shY<<endl;
                    rrtx.moveRobot(rrtx.finalPath[tmp].x_-rrtx.finalPath[0].x_,
                                   rrtx.finalPath[tmp].y_-rrtx.finalPath[0].y_, rrtx.finalPath[tmp].theta_);
                }
                else{
                    rrtx.moveRobot(0, 0, rrtx.startPoint_.theta_);
                }

                rrtx.addDynamicObstacles("../input/map" + imageName + "/image.txt", i, shX, shY);
                outShifts<<shX<<" "<<shY<<endl;
                double diffTime1 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                startTime = high_resolution_clock::now();
                rrtx.search();
                auto diffTime2 = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
                outTime <<diffTime1<<" "<< diffTime2 << endl;
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
