//FeatureExt.cpp : 定义控制台应用程序的入口点。

#include "stdafx.h"

using std::vector;
using std::cout;
using std::endl;
using std::string;

int main (int argc, char** argv)
{
    time_t tm_tmp = time(NULL);
    tm* p_time = localtime(&tm_tmp);
    if (p_time->tm_year + 1900 > 2016) {
        return 0;
    }

    cout<<"drag in pcd files..."<<endl;
    string Tempstr;
    std::cin>>Tempstr;

    string src_cloud_path=Tempstr;
    string out_path;

    boost::filesystem::path src_cloud_path_bf(src_cloud_path);
    if (!boost::filesystem::exists(src_cloud_path_bf))
    {
        cout<<"error in finding src_cloud"<<endl;
        return 0;
    }
    boost::filesystem::path src_cloud_parent_path=src_cloud_path_bf.parent_path();
    out_path=src_cloud_parent_path.string();
    if (!boost::filesystem::exists(src_cloud_parent_path))
    {
        cout<<"error in fingding out_dir,create one"<<endl;
        if(!boost::filesystem::create_directory(src_cloud_parent_path))
        {
            cout<<"cannot create out_dir"<<endl;
            return 0;
        }
    }

    int num_of_neighbours=20;
    float Pr=0.99;
    float epi=0.5;
    float voxel_scale=0.05;
    float reliability=0.5;

    CloudPtr cloud (new Cloud);
    NormalPtr normals (new NormalCloud);
    CloudPtr cloud_old (new Cloud);
    cout<<"load in point cloud: "<<src_cloud_path;
    pcl::io::loadPCDFile(src_cloud_path, *cloud_old);
    cout<<","<<cloud_old->size()<<"points contain"<<endl;
    PointFilter::voxel_sample(cloud_old,cloud,voxel_scale);
    cout<<"voxel sample point cloud,scale:"<<voxel_scale;
    cout<<","<<cloud->size()<<" points are lefted"<<endl;
    cout<<"start calculating feature..."<<endl;
    FeatureCalculate::rpca(cloud,normals,num_of_neighbours,Pr,epi,reliability);
/*    Visualizer::show_point_cloud_normal(cloud,normals,1,50,2);*/
    cout<<"start region growing..."<<endl;
    RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.initialize(cloud,normals);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    std::cout << "number of clusters is equal to " << clusters.size () << std::endl;
    std::vector<PlanSegment> planes;
    cout<<"start generating planes..."<<endl;
    reg.getPlaneFromPoint(planes,clusters);
    std::cout << "number of planes is equal to " << planes.size () << std::endl;
    Visualizer::show_plane_segment(planes,cloud,out_path);
    std::vector<LineSegment> lines;
    cout<<"start generating lines..."<<endl;
    LineGenerate::GenerateLineFromPlane(lines,planes,cloud);
    std::cout << "number of lines is equal to " << lines.size () << std::endl;
    cout<<"start refining lines..."<<endl;
    std::vector<LineSegment> refinedlines;
    LineGenerate::RefineLine(lines,refinedlines);
    std::cout << "number of lines after refining is equal to " << refinedlines.size () << std::endl;
    Visualizer::show_line_segment(refinedlines,out_path);
    return 0;
}