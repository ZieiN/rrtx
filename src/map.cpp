#include "../include/map.h"
#include "../include/Dubins/dubins.h"
#include <cmath>
#include <set>
#include <algorithm>
#include <iostream>

using namespace std;
Map::Map(){
    width_ = 1;
    height_ = 1;
    cell_ = new bool *[1];
    cell_[0] = new bool[1];
//    agent_ = AgentController();
}
Map::~Map(){
    for(int i=0; i<height_; ++i){
      if(width_>0)
        delete cell_[i];
    }
    delete cell_;
//    delete agent_;
}
Map::Map(int height, int width){
	width_ = width;
	height_ = height;
	cell_ = new bool*[height_];
	for(int i = 0; i < height_; ++i)
	{
          cell_[i] = new bool[width_];
	}
//    agent_ = new AgentController;
//    agent_ = AgentController();
}
Map::Map(int width, int height, bool **cell){
    width_ = width;
    height_ = height;
    cell_ = new bool*[height_];
    for(int i = 0; i < height_; ++i)
    {
        cell_[i] = new bool[width_];
        for(int j=0; j<width; ++j){
            cell_[i][j] = cell[i][j];
        }
    }
//    agent_ = new AgentController;
//    agent_ = AgentController();
}
Map::Map(const Map &other)
{
    width_ = height_ = 0;
    setWidth(other.width_);
    setHeight(other.height_);
    for(int i=0; i<height_; ++i){
        for(int j=0; j<width_; ++j){
            cell_[i][j] = other.cell_[i][j];
        }
    }
}
Map &Map::operator=(const Map &mp){
    if(this!=&mp) {
        setMap(mp.getWidth(), mp.getHeight(), mp.getMap());
    }
    return *this;
}
void Map::setMap(int width, int height, bool ** cell){
    setWidth(width);
    setHeight(height);
    for(int i=0; i<height; ++i){
        for(int j=0; j<width; ++j){
            cell_[i][j] = cell[i][j];
        }
    }
}
int Map::getWidth() const{
	return width_;
}
void Map::setWidth(int value){
    for(int i = 0; i < height_; ++i)
    {
//        if(cell_[i]!=nullptr)
        if(width_>0)
          delete  cell_[i];
        cell_[i] = new bool[value];
    }
	width_ = value;
}
int Map::getHeight() const{
	return height_;
}
void Map::setHeight(int value){
    for(int i = 0; i < height_; ++i)
    {
//        if(cell_[i]!=nullptr)
        if(width_>0)
        delete  cell_[i];
    }
    if(height_>0)
      delete cell_;
    cell_ = new bool * [value];
    for(int i = 0; i < value; ++i)
    {
      if(width_>0)
        cell_[i] = new bool[width_];
    }
    height_ = value;
}
bool **Map::getMap() const {
    return cell_;
}

void Map::setCell(int i, int j, bool value){
    cell_[i][j]=value;
}
void Map::setCellXY(int x, int y, bool value){
    cell_[height_-1-y][x]=value;
}
bool Map::cellIsObstacle(int x, int y) const{
    return x < 0 || x >= width_ || y < 0 || y >= height_ || cell_[height_-1-y][x];
}
//bool Map::cellIsObstacle(int x, int y) const{
//    int dx[8] = {0,0,1,1,1,-1,-1, -1};
//    int dy[8] = {1,-1,0,1,-1, 0, 1, -1};
//    for(int i=0; i<8; ++i){
//        if(cellIsObstacle1(x+dx[i], y+dy[i]))
//            return true;
//    }
//    return false;
//}
bool Map::getValue(int i, int j) const {
    return cell_[i][j];
}

bool Map::isIn(int i, int j) {
    return i >= 0 && i < width_ && j >= 0 && j < height_;
}

bool Map::cellIsObstacle(double x, double y) const {
    set<int> sx={static_cast<int>(lround(x+EPS_DOUBLE)), static_cast<int>(lround(x-EPS_DOUBLE))}, sy={static_cast<int>(lround(y+EPS_DOUBLE)), static_cast<int>(lround(y-EPS_DOUBLE))};
    bool ans = false;
    for(auto xx:sx){
        for(auto yy:sy){
            ans = ans || cellIsObstacle(xx, yy);
        }
    }
    return ans;
//    return cellIsObstacle(int(x+0.5), int(y+0.5)) && cellIsObstacle(int(x+EPS_DOUBLE+0.5))
}

bool Map::isInXY(double x, double y) {
    return isIn((height_-1-lround(y)), lround(x));
}

bool Map::cellIsObstacleIJ(int i, int j) const {
    return cell_[i][j];
}
