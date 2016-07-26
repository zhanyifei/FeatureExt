#pragma once
#include "stdafx.h"

class Visualizer
{
public:
    Visualizer(void);
    ~Visualizer(void);
    static void show_point_cloud_normal(CloudPtr cloud,NormalPtr normal,int point_size,int show_level,int normal_size);
    static void show_point_cloud_curvature(CloudPtr cloud,NormalPtr normal,int point_size);
    static void show_line_segment(const vector<LineSegment>& lines,std::string out_path);
    static void show_plane_segment(vector<PlanSegment>& planes,CloudPtr cloud,std::string out_path);
};

