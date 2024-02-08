//
// Created by zain on 08.12.2021.
//

#include "../include/point.h"

bool operator < (Point a, Point b){
    return a.x_<b.x_;
}


std::ostream &operator<<(std::ostream &os, const Point &p) {
    os<<"("<<p.x_<<","<<p.y_<<")";
    return os;
}
