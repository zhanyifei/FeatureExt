#include "StdAfx.h"
#include "feature_calculate.h"

using std::cout;
using std::endl;
using std::sort;
using std::vector;

FeatureCalculate::FeatureCalculate(void)
{
}


FeatureCalculate::~FeatureCalculate(void)
{
}
bool CmpDis(PlanePoint& a, PlanePoint& b)
{
    if (a.dis < b.dis) {
        return true;
    } else {
        return false;
    }
}

bool Cmpmin_value(PlanSegment& a, PlanSegment& b)
{
    if (a.min_value < b.min_value) {
        return true;
    } else {
        return false;
    }
}
int compute_iteration_number(float Pr, float epi, int h_free)
{
    int iteration_number;
    iteration_number = log10(1 - Pr) / log10(1 - pow(1 - epi, 3));
    return iteration_number;
}
float FeatureCalculate::compute_distance_from_point_to_plane(CloudItem& a_point, PlanSegment& plane)
{
    /*计算邻域点到种子点所在平面的垂直距离;*/
    float x1, y1, z1,a,b,c,d;
    //平面外一点（x1,y1,z1）;
    x1 = a_point.x;
    y1 = a_point.y;
    z1 = a_point.z;
    a=plane.normal.normal_x;
    b=plane.normal.normal_y;
    c=plane.normal.normal_z;
    d=plane.distance;
    float dis;
    double g = sqrt(a * a + b * b + c * c);//求平面方程系数nx,ny,nz的平方和的开平方;
    double f1 = a * x1;
    double f2 = b * y1;
    double f3 = c * z1;
    double f4 = d;
    double f = std::abs(f1 + f2 + f3 + f4);
    dis = (f / g);
    return dis;
}
void FeatureCalculate::pca(CloudPtr cloud,PlanSegment &plane)
{    
    Eigen::MatrixXf X(3,cloud->size());
    double avr_x=0.0,avr_y=0.0,avr_z=0.0;
    for (int i = 0; i <cloud->size(); i++) {
        avr_x = avr_x  * double(i) / (i + 1) + cloud->points[i].x / double(i + 1);
        avr_y = avr_y  * double(i) / (i + 1) + cloud->points[i].y / double(i + 1);
        avr_z = avr_z  * double(i) / (i + 1) + cloud->points[i].z / double(i + 1);
    }
    for (int i=0;i<cloud->size();i++) 
    {
        X(0,i)=cloud->points[i].x-avr_x;
        X(1,i)=cloud->points[i].y-avr_y;
        X(2,i)=cloud->points[i].z-avr_z;
    }
    Eigen::MatrixXf XT(cloud->size(),3); 
    XT=X.transpose();
    Eigen::Matrix3f XXT; 
    XXT=X*XT;
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    pcl::eigen33 (XXT, eigen_value, eigen_vector);
    plane.normal.normal_x = eigen_vector [0];
    plane.normal.normal_y = eigen_vector [1];
    plane.normal.normal_z = eigen_vector [2];
    plane.distance = - (plane.normal.normal_x * avr_x + plane.normal.normal_y * avr_y + plane.normal.normal_z * avr_z);

    float eig_sum = XXT.coeff (0) + XXT.coeff (4) + XXT.coeff (8);
    plane.min_value=eigen_value;
    if (eig_sum != 0)
    plane.normal.curvature = fabsf (eigen_value / eig_sum);
    else
        plane.normal.curvature = 0.0;
}
void FeatureCalculate::rpca_plane(CloudPtr cloud, PlanSegment &plane_rpca, float Pr, float epi,float reliability)
{
    CloudPtr cloud_h(new Cloud);
    CloudPtr cloud_final(new Cloud);
    int iteration_number;
    int h_free;
    int point_num=cloud->size();
    vector<PlanePoint> plane_points;
    plane_points.resize(point_num);
    h_free = int(reliability * point_num);
    iteration_number = compute_iteration_number(Pr, epi, h_free);
    vector<PlanSegment> planes;
    for (int i = 0; i < iteration_number; i++) {
        CloudPtr points(new Cloud);
        int num[3];
        for (int n = 0; n < 3; n++) {
            num[n] = rand() % point_num;
            points->push_back(cloud->points[num[n]]);
        }
        if (num[0] == num[1] || num[0] == num[2] || num[1] == num[2])
            continue;
        PlanSegment a_plane;
        pca(points,a_plane);
        for (int n = 0; n < point_num; n++) {
            plane_points[n].dis =compute_distance_from_point_to_plane(cloud->points[n],a_plane);
            plane_points[n].point=cloud->points[n];
        }
        std::sort(plane_points.begin(), plane_points.end(), CmpDis);
        for (int n = 0; n < h_free; n++) {
            CloudItem temp_point=plane_points[n].point;
            cloud_h->points.push_back(temp_point);
        }
        pca(cloud_h,a_plane);
        cloud_h->points.clear();
        planes.push_back(a_plane);
    }
    sort(planes.begin(), planes.end(), Cmpmin_value);
    PlanSegment final_plane;
    final_plane = planes[0];
    planes.clear();
//     vector<float> distance;
//     vector<float> distance_sort;
//     for (int n = 0; n < point_num; n++) {
//         float dis_from_point_plane;
//         dis_from_point_plane =compute_distance_from_point_to_plane(cloud->points[n],final_plane);
//         distance.push_back(dis_from_point_plane);
//         distance_sort.push_back(dis_from_point_plane);
//     }
//     sort(distance_sort.begin(), distance_sort.end());
//     float distance_median;
//     distance_median = distance_sort[point_num / 2];
//     distance_sort.clear();
//     float MAD;
//     vector<float> temp_MAD;
//     for (int n = 0; n < point_num; n++) {
//         temp_MAD.push_back(abs(distance[n] - distance_median));
//     }
//     sort(temp_MAD.begin(), temp_MAD.end());
//     MAD = 1.4826 * temp_MAD[point_num / 2];
//  /*   pcl::PointIndices points_id;*/
//     if (MAD == 0) {
//         for (int n = 0; n < point_num; n++) {
//             CloudItem temp_point=cloud->points[n];
//             cloud_final->points.push_back(temp_point);
//         }
//     } else {
//         for (int n = 0; n < point_num; n++) {
//             float Rz;
//             Rz = (abs(distance[n] - distance_median)) / MAD;
//             if (Rz < 2.5) {
//                 CloudItem temp_point=cloud->points[n];
//                 cloud_final->points.push_back(temp_point);
/*                points_id.indices.push_back(plane_rpca.points_id.indices[n]);*/
//             }
//         }
//     }
/*    plane_rpca.points_id=points_id;*/
/*   pca(cloud_final,plane_rpca);*/
    final_plane.points_id=plane_rpca.points_id;
    plane_rpca=final_plane;
}

void FeatureCalculate::rpca(CloudPtr cloud,NormalPtr normals, int num_of_neighbors, float Pr, float epi,float reliability)
{
    pcl::KdTreeFLANN<CloudItem> kdtree;
    kdtree.setInputCloud(cloud);
    normals->resize(cloud->size());
    #pragma omp parallel for
    for (int j = 0; j < cloud->size(); j++) {
        CloudPtr cloud_h(new Cloud);
        CloudPtr cloud_final(new Cloud);
        vector<int> point_id_search;
        vector<float> point_distance;
        int point_num = kdtree.nearestKSearch(j,num_of_neighbors,point_id_search,point_distance);
        point_distance.clear();
        vector<PlanePoint> plane_points;
        plane_points.resize(point_num);
        if (point_num > 3) {
            int iteration_number;
            int h_free;
            h_free = int(reliability * point_num);
            iteration_number = compute_iteration_number(Pr, epi, h_free);
            vector<PlanSegment> planes;
            for (int i = 0; i < iteration_number; i++) {
                CloudPtr points(new Cloud);
                int num[3];
                for (int n = 0; n < 3; n++) {
                    num[n] = rand() % point_num;
                    points->push_back(cloud->points[point_id_search[num[n]]]);
                }
                if (num[0] == num[1] || num[0] == num[2] || num[1] == num[2])
                    continue;
                PlanSegment a_plane;
                pca(points,a_plane);
                for (int n = 0; n < point_num; n++) {
                    plane_points[n].dis =compute_distance_from_point_to_plane(cloud->points[point_id_search[n]],a_plane);
                    plane_points[n].point=cloud->points[point_id_search[n]];
                }
                std::sort(plane_points.begin(), plane_points.end(), CmpDis);
                for (int n = 0; n < h_free; n++) {
                    CloudItem temp_point=plane_points[n].point;
                    cloud_h->points.push_back(temp_point);
                }
                pca(cloud_h,a_plane);
                cloud_h->points.clear();
                planes.push_back(a_plane);
            }
            sort(planes.begin(), planes.end(), Cmpmin_value);
            PlanSegment final_plane;
            final_plane = planes[0];
            planes.clear();
            vector<float> distance;
            vector<float> distance_sort;
            for (int n = 0; n < point_num; n++) {
                float dis_from_point_plane;
                dis_from_point_plane =compute_distance_from_point_to_plane(cloud->points[point_id_search[n]],final_plane);
                distance.push_back(dis_from_point_plane);
                distance_sort.push_back(dis_from_point_plane);
            }
            sort(distance_sort.begin(), distance_sort.end());
            float distance_median;
            distance_median = distance_sort[point_num / 2];
            distance_sort.clear();
            float MAD;
            vector<float> temp_MAD;
            for (int n = 0; n < point_num; n++) {
                temp_MAD.push_back(abs(distance[n] - distance_median));
            }
            sort(temp_MAD.begin(), temp_MAD.end());
            MAD = 1.4826 * temp_MAD[point_num / 2];
            if (MAD == 0) {
                for (int n = 0; n < point_num; n++) {
                    CloudItem temp_point=cloud->points[point_id_search[n]];
                    cloud_final->points.push_back(temp_point);
                }
            } else {
                for (int n = 0; n < point_num; n++) {
                    float Rz;
                    Rz = (abs(distance[n] - distance_median)) / MAD;
                    if (Rz < 2.5) {
                        CloudItem temp_point=cloud->points[point_id_search[n]];
                        cloud_final->points.push_back(temp_point);
                    }
                }
            }
            point_id_search.clear();
            if (cloud_final->points.size() > 3)
            {
                pca(cloud_final,final_plane);
                normals->points[j]= final_plane.normal;
            } else {
                normals->points[j].normal_x = 0.0;
                normals->points[j].normal_y = 0.0;
                normals->points[j].normal_z = 0.0;
                normals->points[j].curvature = 1.0;
            }
            cloud_final->clear();
        } else {
            normals->points[j].normal_x = 0.0;
            normals->points[j].normal_y = 0.0;
            normals->points[j].normal_z = 0.0;
            normals->points[j].curvature = 1.0;
        }    
    }
}
