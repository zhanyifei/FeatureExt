#include "StdAfx.h"
#include "line_generate.h"


LineGenerate::LineGenerate(void)
{
}


LineGenerate::~LineGenerate(void)
{
}

void LineGenerate::GenerateLineFromPlane(vector<LineSegment>& linesegments,const vector<PlanSegment>& planes,const CloudPtr &cloud)
{
    int max_points_mun=0;
    LineSegment main_line;
    for (int i=0;i<planes.size();i++)
    {
        if (max_points_mun<planes[i].points_id.indices.size())
        {
            max_points_mun=planes[i].points_id.indices.size();
            main_line.normal_x=planes[i].normal.normal_x;
            main_line.normal_y=planes[i].normal.normal_y;
        }
    }
    for (int i=0;i<planes.size();i++)  //主方向矫正
    {
        PlanSegment plane=planes[i];
        LineSegment linesegment;
        linesegment.normal_x=plane.normal.normal_x;
        linesegment.normal_y=plane.normal.normal_y;
        linesegment.distance=plane.distance;
        float dot_=plane.normal.normal_x*main_line.normal_x+plane.normal.normal_y*main_line.normal_y;
        if (std::fabs(dot_)>0.92)  //平行 10度
        {
            linesegment.normal_x=main_line.normal_x/main_line.normal_y;
            linesegment.normal_y=1.0;
            linesegment.distance=-linesegment.normal_x*cloud->points[plane.points_id.indices[0]].x-linesegment.normal_y*cloud->points[plane.points_id.indices[0]].y;
        }
        if (std::fabs(dot_)<0.08)  //垂直 10度
        {
            linesegment.normal_x=-main_line.normal_y/main_line.normal_x;
            linesegment.normal_y=1.0;
            linesegment.distance=-linesegment.normal_x*cloud->points[plane.points_id.indices[0]].x-linesegment.normal_y*cloud->points[plane.points_id.indices[0]].y;
        }
        CloudPtr tempcloud(new Cloud);
        for (auto iter = plane.points_id.indices.begin(); iter < plane.points_id.indices.end(); ++iter)
        {
            tempcloud->push_back(cloud->points.at(*iter));
        }
        CloudItem minPt, maxPt;
        pcl::getMinMax3D (*tempcloud,minPt,maxPt);
        GetStartEndpt(linesegment,minPt,maxPt);
        linesegments.push_back(linesegment);
    }
}
void  LineGenerate::GetStartEndpt(LineSegment &linesegment,CloudItem minPt,CloudItem maxPt)  //求线段端点
{
    pcl::PointXYZ pt1,pt2,_pt1,_pt2,startpoint,endpoint;
    pt1.x=minPt.x,pt1.y=maxPt.y;
    pt2.x=maxPt.x,pt2.y=minPt.y;
    _pt1.x=maxPt.x,_pt1.y=maxPt.y;
    _pt2.x=minPt.x,_pt2.y=minPt.y;

    pcl::PointXYZ startpt,endpt,_startpt,_endpt;
    float A,B,C;
    A=linesegment.normal_x;
    B=linesegment.normal_y;
    C=linesegment.distance;

    startpoint=pt1;endpoint=pt2;
    startpt.x=(B*B*startpoint.x-A*B*startpoint.y-A*C)/(A*A+B*B);
    startpt.y=(A*A*startpoint.y-A*B*startpoint.x-B*C)/(A*A+B*B);
    startpt.z=0.0;
    endpt.x=(B*B*endpoint.x-A*B*endpoint.y-A*C)/(A*A+B*B);
    endpt.y=(A*A*endpoint.y-A*B*endpoint.x-B*C)/(A*A+B*B);
    endpt.z=0.0;
    float dis1=(startpt.x-endpt.x)*(startpt.x-endpt.x)+(startpt.y-endpt.y)*(startpt.y-endpt.y);
    startpoint=_pt1;endpoint=_pt2;
    _startpt.x=(B*B*startpoint.x-A*B*startpoint.y-A*C)/(A*A+B*B);
    _startpt.y=(A*A*startpoint.y-A*B*startpoint.x-B*C)/(A*A+B*B);
    _startpt.z=0.0;
    _endpt.x=(B*B*endpoint.x-A*B*endpoint.y-A*C)/(A*A+B*B);
    _endpt.y=(A*A*endpoint.y-A*B*endpoint.x-B*C)/(A*A+B*B);
    _endpt.z=0.0;
    float dis2=(_startpt.x-_endpt.x)*(_startpt.x-_endpt.x)+(_startpt.y-_endpt.y)*(_startpt.y-_endpt.y);;
    if (dis1>dis2)
    {
        linesegment.startpt=startpt;
        linesegment.endpt=endpt;
    } 
    else
    {
        linesegment.startpt=_startpt;
        linesegment.endpt=_endpt;
    }
}
void LineGenerate::RefineLine(vector<LineSegment>& src_line,vector<LineSegment>& dst_line,Config &configs)
{
    CLineMatcher line_matcher;
    line_matcher.initialize(configs.max_gap,configs.max_slope,configs.max_distance,configs.max_length,configs.extension);
    std::vector<cv::Vec4i> lines;
    for (int i=0;i<src_line.size();i++)
    {
        cv::Vec4i line;
        line[0]=(int)(src_line[i].startpt.x*100.0);
        line[1]=(int)(src_line[i].startpt.y*100.0);
        line[2]=(int)(src_line[i].endpt.x*100.0);
        line[3]=(int)(src_line[i].endpt.y*100.0);
        lines.push_back(line);
    }
    std::vector<cv::Vec4i> match_lines = line_matcher.growing_line(lines);
    for (int i=0;i<match_lines.size();i++)
    {
        LineSegment line_s;
        line_s.startpt.x=(float)match_lines[i][0]/100.0;
        line_s.startpt.y=(float)match_lines[i][1]/100.0;
        line_s.endpt.x=(float)match_lines[i][2]/100.0;
        line_s.endpt.y=(float)match_lines[i][3]/100.0;
        dst_line.push_back(line_s);
    }
}


