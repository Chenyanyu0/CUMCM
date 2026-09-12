
#pragma once
#include <vector>

struct Point{
    double x=0;
    double y=0;
};

struct Circle{
    Point center;
    double radius=0;
};

struct Target{
    int channel;
    Point firstPoint;
    double firstAngle;
    Point estimate;
    std::vector<Point> region;
    Circle bound;
    bool cleared=false;
};

struct Statistics{
    int measureCount=0;
    int clearCount=0;
    double moveDistance=0;
    double virtualTime=0;
};
