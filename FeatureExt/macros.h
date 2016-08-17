#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ CloudItem;//XYZ格式的点云单元
typedef pcl::PointCloud<CloudItem> Cloud;//由XYZA格式的点云单元构成的点云数据
typedef Cloud::ConstPtr CloudConstPtr;//指向点云的常量指针
typedef Cloud::Ptr CloudPtr;//指向点云的指针,点云pcl中Ptr为shared_ptr类型

typedef pcl::Normal NormalItem;//法向量
typedef pcl::PointCloud<NormalItem> NormalCloud;//法向量构成的点云数据
typedef NormalCloud::ConstPtr NormalConstPtr;//指向法向量点云的常量指针
typedef NormalCloud::Ptr NormalPtr;//指向法向量点云的指针