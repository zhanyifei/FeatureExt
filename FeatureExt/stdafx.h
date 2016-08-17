// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <time.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "macros.h"
#include "data_struct.h"
#include "file_io.h"
#include "feature_calculate.h"
#include "visualizer.h"
#include "point_filter.h"
#include "region_growing.h"
#include "line_generate.h"
#include "MyLine.h"
#include "LineMatcher.h"
#include "ToolFunction.h"


// TODO: 在此处引用程序需要的其他头文件
