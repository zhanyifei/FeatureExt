#include "stdafx.h"
#include "MyLine.h"


CMyLine::CMyLine()
{
}

CMyLine::CMyLine(const cv::Vec4i& line)
{
	m_line_vec4i[0] = line[0];
	m_line_vec4i[1] = line[1];
	m_line_vec4i[2] = line[2];
	m_line_vec4i[3] = line[3];

	pt_start.x = line[0];
	pt_start.y = line[1];
	pt_end.x = line[2];
	pt_end.y = line[3];

	if (pt_start.y > pt_end.y)
	{
		cv::Point pt_temp = pt_start;
		pt_start = pt_end;
		pt_end = pt_temp;
		m_line_vec4i[0] = pt_start.x;
		m_line_vec4i[1] = pt_start.y;
		m_line_vec4i[2] = pt_end.x;
		m_line_vec4i[3] = pt_end.y;
	}

	pt_center.x = pt_start.x - (int)((pt_start.x - pt_end.x) / 2);
	pt_center.y = pt_start.y - (int)((pt_start.y - pt_end.y) / 2);

	length = (float)std::sqrt(std::pow((float)std::abs(pt_start.x - pt_end.x), 2) + std::pow((float)std::abs(pt_start.y - pt_end.y), 2));
	k = (float)(std::abs(pt_start.y - pt_end.y)) / (float)(std::abs(pt_start.x - pt_end.x) + FLT_EPSILON);

	A = (float)(pt_end.y - pt_start.y);
	B = (float)(pt_start.x - pt_end.x);
	C = (float)(pt_end.x*pt_start.y - pt_start.x*pt_end.y);

	is_delete = false;
}


CMyLine::~CMyLine()
{
}
