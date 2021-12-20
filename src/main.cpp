#include <bits/stdc++.h>
#include "../include/rrtx.h"
#include "../include/constants.h"

using namespace std;
using namespace std::chrono;

int main() {
    srand(0);
    for(int imageNum = 1; imageNum<=3; ++imageNum) {
        string imageName = to_string(imageNum);
        string inputsFile = "../input/map" + imageName + "/inputs.txt";
        ifstream in;
        string timesFile = "../output/map" + imageName + "/Dynamics/times.txt";
        ofstream outTime;
        in.open(inputsFile.c_str());
        outTime.open(timesFile.c_str());
        double stx, sty, sto, ndx, ndy, ndo;
        int cnt = 0;
        while (in >> stx >> sty >> sto >> ndx >> ndy >> ndo) {
            Point start = Point(stx, sty, sto);
            Point goal = Point(ndx, ndy, ndo);
            RRTX rrtx(start, goal);
            rrtx.updateMapFromFile("../input/map" + imageName + "/image.txt");
            for (int i = 0; i < 50; ++i) {
//            rrtx.moveObstacles("../input/map"+imageName+"/image.txt", i);
                auto startTime = high_resolution_clock::now();
                rrtx.addDynamicObstacles("../input/map" + imageName + "/image.txt", i);
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
//            break;
            ++cnt;
        }
//        break;
    }
    return 0;
}
