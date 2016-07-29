#include "stdafx.h"
#include "LineMatcher.h"
#include "ToolFunction.h"
#include <math.h>


CLineMatcher::CLineMatcher()
{
}


CLineMatcher::~CLineMatcher()
{
}

bool CompareLength(CMyLine& a, CMyLine& b)
{
    if (a.length > b.length) {
        return true;
    } else {
        return false;
    }
}
void CLineMatcher::loadLines(const std::vector<cv::Vec4i>& lines)
{
    m_my_lines.clear();
    for (size_t i = 0; i < lines.size(); i++)
    {
        CMyLine my_line(lines[i]);
        m_my_lines.push_back(my_line);
    }
}
CMyLine CLineMatcher::MergeLine(CMyLine& _startline,CMyLine& _endline)
{
    CMyLine main_line;;
    if (_startline.length>_endline.length)
    {
        main_line=_startline;
    }
    else
        main_line=_endline;
    cv::Point pt1,pt2,pt3,pt4;
    pt1=CToolFunction::getPedal(_startline.pt_start,main_line);
    pt2=CToolFunction::getPedal(_startline.pt_end,main_line);
    pt3=CToolFunction::getPedal(_endline.pt_start,main_line);
    pt4=CToolFunction::getPedal(_endline.pt_end,main_line);
    vector<double> dis;
    double dis1,dis2,dis3,dis4,dis5,dis6; //两两间距离
    dis1=CToolFunction::getPointDistance(pt1,pt2);
    dis2=CToolFunction::getPointDistance(pt1,pt3);
    dis3=CToolFunction::getPointDistance(pt1,pt4);
    dis4=CToolFunction::getPointDistance(pt2,pt3);
    dis5=CToolFunction::getPointDistance(pt2,pt4);
    dis6=CToolFunction::getPointDistance(pt3,pt4);
    dis.push_back(dis1);
    dis.push_back(dis2);
    dis.push_back(dis3);
    dis.push_back(dis4);
    dis.push_back(dis5);
    dis.push_back(dis6);
    sort(dis.begin(),dis.end());
    double maxdis=dis[6];
    if((maxdis-dis1)<DBL_MIN)
    {
        cv::Vec4i merge_line = cv::Vec4i(pt1.x, pt1.y, pt2.x, pt2.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
    else if((maxdis-dis2)<DBL_MIN)
    {
        cv::Vec4i merge_line = cv::Vec4i(pt1.x, pt1.y, pt3.x, pt3.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
    else if((maxdis-dis3)<DBL_MIN)
    {
        cv::Vec4i merge_line = cv::Vec4i(pt1.x, pt1.y, pt4.x, pt4.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
    else if((maxdis-dis4)<DBL_MIN)
    {
        cv::Vec4i merge_line = cv::Vec4i(pt2.x, pt2.y, pt3.x, pt3.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
    else if((maxdis-dis5)<DBL_MIN)
    {
        cv::Vec4i merge_line = cv::Vec4i(pt2.x, pt2.y, pt4.x, pt4.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
    else
    {
        cv::Vec4i merge_line = cv::Vec4i(pt3.x, pt3.y, pt3.x, pt3.y);
        CMyLine mergeline(merge_line);
        return mergeline;
    }
}
bool CLineMatcher::IsMerge(CMyLine& _startline,CMyLine& _endline)
{
    if (std::abs(std::atan2(1,_startline.k)-std::atan2(1,_endline.k))<m_max_slope)   //平行
    {
        float dis=CToolFunction::getParallelLineDistance(_startline,_endline);
        if (dis<150)//平行线间距离1.5
        {
            double mindis;
            vector<double> dis;
            double dis1,dis2,dis3,dis4;
            dis1=CToolFunction::getPointDistance(_startline.pt_start,_endline.pt_start);
            dis2=CToolFunction::getPointDistance(_startline.pt_start,_endline.pt_end);
            dis3=CToolFunction::getPointDistance(_startline.pt_end,_endline.pt_start);
            dis4=CToolFunction::getPointDistance(_startline.pt_end,_endline.pt_end);
            dis.push_back(dis1);
            dis.push_back(dis2);
            dis.push_back(dis3);
            dis.push_back(dis4);
            sort(dis.begin(),dis.end());
            mindis=dis[0];
            float dot1=(_startline.pt_start.x-_endline.pt_end.x)*(_startline.pt_end.x-_endline.pt_end.x)+
                (_startline.pt_start.y-_endline.pt_end.y)*(_startline.pt_end.y-_endline.pt_end.y);
            float dot2=(_startline.pt_start.x-_endline.pt_start.x)*(_startline.pt_end.x-_endline.pt_start.x)+
                (_startline.pt_start.y-_endline.pt_start.y)*(_startline.pt_end.y-_endline.pt_start.y);
            if (mindis>m_max_gap&&(dot1>=0.0)&&(dot2>=0.0))
            {
                return false;
            }else
                return true;
        }
        else
            return false;
    }
    else
        return false;
}
std::vector<cv::Vec4i> CLineMatcher::matchMyLines(const std::vector<cv::Vec4i>& lines)
{
    loadLines(lines);
    m_match_once_lines.clear();
    sort(m_my_lines.begin(),m_my_lines.end(),CompareLength);
    for (size_t i = 0; i < m_my_lines.size(); i++)
    {
        if (!m_my_lines[i].is_delete)
        {
            //m_my_lines[i].setDelete(true);
            for (size_t j = 0; j < m_my_lines.size(); j++)
            {
                if (!m_my_lines[j].is_delete)
                {
                    if (IsMerge(m_my_lines[i],m_my_lines[j]))
                    {
                        m_my_lines[i]=MergeLine(m_my_lines[i],m_my_lines[j]);
                        m_my_lines[j].setDelete(true);
                    }
                    else
                        continue;
                }
            } 
            if(m_my_lines[i].length>200)//2米的线
            {
                CMyLine mergeline=extend_line(m_my_lines[i],100.0);  //两端延长一米
                m_match_once_lines.push_back(mergeline.m_line_vec4i);
                //m_match_once_lines.push_back(m_my_lines[i].m_line_vec4i);
            }
        }
    } 
    return m_match_once_lines;
}

void CLineMatcher::loadDebugImage(const cv::Mat& draw_img)
{
    draw_img.copyTo(m_debug_img);
}

CMyLine CLineMatcher::extend_line(CMyLine &line,float extension) 
{
    cv::Vec2i start_vec,end_vec;
    start_vec[0]=line.m_line_vec4i[0]-line.m_line_vec4i[2];
    start_vec[1]=line.m_line_vec4i[1]-line.m_line_vec4i[3];
    end_vec[0]=line.m_line_vec4i[2]-line.m_line_vec4i[0];
    end_vec[1]=line.m_line_vec4i[3]-line.m_line_vec4i[1];
    float scale=extension/line.length;
    start_vec[0]*=scale;
    start_vec[1]*=scale;
    end_vec[0]*=scale;
    end_vec[1]*=scale;
    CMyLine new_line;
    cv::Vec4i new_line_vec = cv::Vec4i(line.m_line_vec4i[0]+start_vec[0], line.m_line_vec4i[1]+start_vec[1], line.m_line_vec4i[2]+end_vec[0], line.m_line_vec4i[3]+end_vec[1]);
    CMyLine mergeline(new_line_vec);
    return mergeline;
}