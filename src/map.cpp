#include "../include/map.h"
#include "../include/Dubins/dubins.h"
#include <cmath>
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
bool Map::isThereLineOfSight(int x0, int y0, int x1, int y1) const
{
    int steep = abs(y1 - y0) > abs(x1 - x0) ;
    if (steep)
    {
        swap(x0, y0);
        swap(x1, y1);
    }
    if (x0 > x1)
    {
        swap(x0, x1);
        swap(y0, y1);
    }

    float dx = x1 - x0;
    float dy = y1 - y0;
    float gradient;
    if (dx == 0.0)
        gradient = 0;
    else
        gradient = dy / dx;
    int xpxl1 = x0;
    int xpxl2 = x1;
    float intersectY = y0 + 0.5;
    if (steep)
    {
        
        if(y1<y0){
            for (int x = xpxl1 ; x < xpxl2 ; x++)
            {
                double realPart = intersectY-int(intersectY);
                if(cellIsObstacle(intersectY, x))
                {
                    return false;
                }
                if(realPart <= abs(gradient) * 0.5 && cellIsObstacle(intersectY-1, x))
                {
                    return false;
                }
                if(realPart >= abs(gradient) * 0.5 && cellIsObstacle(intersectY, x+1))
                {
                    return false;
                }
                intersectY += gradient;
            }
        }
        else{
            for (int x = xpxl1 ; x < xpxl2 ; x++)
            {
                double realPart = intersectY-int(intersectY);
                if(cellIsObstacle(intersectY, x))
                {
                    return false;
                }
                if(1-realPart >= abs(gradient) * 0.5 && cellIsObstacle(intersectY, x+1))
                {
                    return false;
                }
                if(1-realPart <= abs(gradient) * 0.5 && cellIsObstacle(intersectY+1, x))
                {
                    return false;
                }
                intersectY += gradient;
            }
        }
    }
    else
    {
        if(y1<y0){
            for (int x = xpxl1 ; x < xpxl2 ; x++)
            {
                double realPart = intersectY-int(intersectY);
                if(cellIsObstacle(x, intersectY))
                {
                    return false;
                }
                if(realPart <= abs(gradient) * 0.5 && cellIsObstacle(x, intersectY-1))
                {
                    return false;
                }
                if(realPart >= abs(gradient) * 0.5 && cellIsObstacle(x+1, intersectY))
                {
                    return false;
                }
                intersectY += gradient;
            }
        }
        else{
            for (int x = xpxl1 ; x < xpxl2 ; x++)
            {
                double realPart = intersectY-int(intersectY);
                if(cellIsObstacle(x, intersectY))
                {
                    return false;
                }
                if(1-realPart >= abs(gradient) * 0.5 && cellIsObstacle(x+1, intersectY))
                {
                    return false;
                }
                if(1-realPart <= abs(gradient) * 0.5 && cellIsObstacle(x, intersectY+1))
                {
                    return false;
                }
                intersectY += gradient;
            }
        }
    }
    return true;
}

bool Map::getValue(int i, int j) const {
    return cell_[i][j];
}

bool Map::isIn(int x, int y) {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}
