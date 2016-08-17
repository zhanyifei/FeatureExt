#pragma once
#include "stdafx.h"

#include "stdafx.h"


class CLineMatcher
{
public:
	CLineMatcher();
	~CLineMatcher();

	inline void initialize(float max_gap, float max_slope,float max_distance,float max_length,float extension)
	{
		m_max_gap = max_gap*100.0;
        m_max_slope = max_slope/180.0*M_PI;
        m_max_distance = max_distance*100.0;
		m_max_length = max_length*100.0;
        m_extension=extension*100.0;
	}

    CMyLine MergeLine(CMyLine& _startline,CMyLine& _endline);
    CMyLine MergeLine(std::vector<int> num_lines);
    bool IsMerge(CMyLine& _startline,CMyLine& _endline);
    CMyLine extend_line(CMyLine &line,float extension);
    std::vector<cv::Vec4i> growing_line(const std::vector<cv::Vec4i>& lines);
    int growRegion(int initial_seed);

	
private:
	void loadLines(const std::vector<cv::Vec4i>& lines);

	float m_max_gap;
	float m_max_slope;
    float m_max_distance;
    float m_max_length;
    float m_extension;
	std::vector<CMyLine> m_my_lines;
	std::vector<cv::Vec4i> m_match_once_lines;
};

