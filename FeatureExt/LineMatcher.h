#pragma once
#include "stdafx.h"

#include "stdafx.h"


class CLineMatcher
{
public:
	CLineMatcher();
	~CLineMatcher();

	inline void setMaxGapAndSlope(float max_gap, float max_slope)
	{
		m_max_gap = max_gap;
		m_max_slope = max_slope;
	}

	std::vector<cv::Vec4i> matchMyLines(const std::vector<cv::Vec4i>& lines);
	void loadDebugImage(const cv::Mat& draw_img);
    CMyLine MergeLine(CMyLine& _startline,CMyLine& _endline);
    bool IsMerge(CMyLine& _startline,CMyLine& _endline);
    CMyLine extend_line(CMyLine &line,float extension);
	
private:
	void loadLines(const std::vector<cv::Vec4i>& lines);

	float m_max_gap;
	float m_max_slope;
	std::vector<CMyLine> m_my_lines;
	std::vector<cv::Vec4i> m_match_once_lines;
	cv::Mat m_debug_img;
};

