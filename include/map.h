#pragma once
#include "../include/Dubins/dubins.h"
//#include "../include/Dubins/Agent.h"
class Map{
public:
	Map();
	~Map();
	Map (int, int);
	Map (int, int, bool**);
	Map (const Map&);
	Map &operator=(const Map&);
	int getWidth() const;
	void setWidth(int value);
	int getHeight() const;
	void setHeight(int value);
	bool ** getMap() const;
	void setMap(int, int, bool**);
	void setCell(int i, int j, bool value);
	bool cellIsObstacle(int x, int y) const;
    bool cellIsObstacle1(int x, int y) const;
	bool isThereLineOfSight(int x0, int y0, int x1, int y1) const;
    bool getValue(int i, int j) const;

    bool isIn(int x, int y);
    void setCellXY(int x, int y, bool value);

//    AgentController agent_;
private:
 	int width_, height_;
	bool **cell_;
};