#pragma once
#include "stdafx.h"


class CToolFunction
{
public:
	CToolFunction();
	~CToolFunction();

	static float getPointDistance(cv::Point pt1, cv::Point pt2);
	static float getPointToLineDistance(cv::Point pt, CMyLine my_line);
	static float getLineSlope(cv::Vec4i line);
	static float getParallelLineDistance(CMyLine l1, CMyLine l2);	// 按照直线1的斜率进行计算
	static cv::Point2f getTwoLinePointIntersection(CMyLine l1, CMyLine l2);
    static cv::Point getPedal(cv::Point pt,CMyLine line);   //求垂足
};

