#pragma once
#include "stdafx.h"

class CMyLine
{
public:
	CMyLine();
	CMyLine(const cv::Vec4i& line);
	~CMyLine();

	inline void setDelete(bool del)
	{
		is_delete = del;
	}

	cv::Point pt_start;
	cv::Point pt_end;
	cv::Point pt_center;
	float k;
	float length;
	bool is_delete;
	cv::Vec4i m_line_vec4i;

	float A;
	float B;
	float C;
	
};

