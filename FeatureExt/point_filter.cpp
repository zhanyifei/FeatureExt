#include "StdAfx.h"
#include "point_filter.h"


PointFilter::PointFilter(void)
{
}


PointFilter::~PointFilter(void)
{
}

void PointFilter::voxel_sample(CloudPtr src_cloud,CloudPtr dst_cloud,float scale)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(src_cloud);
    voxelSampler.setLeafSize(scale,scale,scale);
    voxelSampler.filter(*dst_cloud);
}
