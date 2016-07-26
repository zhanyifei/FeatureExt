#pragma once
#include "stdafx.h"
#include "data_struct.h"


class FeatureCalculate
{
public:
    FeatureCalculate(void);
    ~FeatureCalculate(void);
    static void pca(CloudPtr cloud,PlanSegment &plane);         //pcaÇóÌØÕ÷Öµ
    static void rpca(CloudPtr cloud, NormalPtr normals, int num_of_neighbors, float Pr, float epi,float reliability);       //Robust pca
    static void rpca_plane(CloudPtr cloud, PlanSegment& plane_rpca, float Pr, float epi,float reliability);
    static float compute_distance_from_point_to_plane(CloudItem& a_point, PlanSegment &plane);
private:

    
};

