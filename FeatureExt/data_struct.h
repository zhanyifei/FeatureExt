#pragma once
#include "macros.h"

struct PlanSegment
{
    uint32_t plane_id;
    pcl::PointIndices points_id;   //面片包含的点
    NormalItem normal;   //normal x,y,z,curvature
    float min_value; //lamda3
    float distance;  //平面方程ax+by+cz+d=0;
};
//归属于某平面的点
struct PlanePoint
{
    CloudItem point;
    float dis;
};
//线段
struct LineSegment
{
    float normal_x;      //直线参数ax+by+z=0
    float normal_y;
    float distance;
    CloudItem startpt,endpt;    //线段起点终点
};