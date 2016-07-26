#include "stdafx.h"
#include "ToolFunction.h"


CToolFunction::CToolFunction()
{
}


CToolFunction::~CToolFunction()
{
}

float CToolFunction::getPointDistance(cv::Point pt1, cv::Point pt2)
{
	return (float)std::sqrt(std::pow((float)std::abs(pt1.x - pt2.x), 2) + std::pow((float)std::abs(pt1.y - pt2.y), 2));
}

float CToolFunction::getPointToLineDistance(cv::Point pt, CMyLine my_line)
{
	return (float)std::abs((float)(my_line.A*pt.x + my_line.B*pt.y + my_line.C) / (float)std::sqrt(std::pow(my_line.A, 2) + std::pow(my_line.B, 2)));
}

float CToolFunction::getLineSlope(cv::Vec4i line)
{
	return (float)(std::abs(line[1] - line[3])) / (float)(std::abs(line[0] - line[2]) + FLT_EPSILON);
}

float CToolFunction::getParallelLineDistance(CMyLine l1, CMyLine l2)
{
	return (float)std::abs(l1.C - l2.C) / (float)std::sqrt(std::pow(l1.A, 2) + std::pow(l1.B, 2));
}

cv::Point2f CToolFunction::getTwoLinePointIntersection(CMyLine l1, CMyLine l2)
{
	float x = (l1.B*l2.C - l2.B*l1.C) / (l1.A*l2.B - l2.A*l1.B);
	float y = (l2.A*l1.C - l1.A*l2.C) / (l1.A*l2.B - l2.A*l1.B);
	return cv::Point2f(x, y);
}
cv::Point CToolFunction::getPedal(cv::Point pt,CMyLine line)
{
    cv::Point pedal;
    pedal.x=(line.B*line.B*pt.x-line.A*line.B*pt.y-line.A*line.C)/(line.A*line.A+line.B*line.B);
    pedal.y=(line.A*line.A*pt.y-line.A*line.B*pt.x-line.B*line.C)/(line.A*line.A+line.B*line.B);
    return pedal;
}

