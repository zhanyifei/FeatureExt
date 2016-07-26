#pragma once
class PointFilter
{
public:
    PointFilter(void);
    ~PointFilter(void);

static void voxel_sample(CloudPtr src_cloud,CloudPtr dst_cloud,float scale);
};

