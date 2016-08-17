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
    if (argc!=4)
    {
        cout<<"error input:input_file_path config_file_path output_path"<<endl;
        return 0;
    }

    string src_cloud_path=argv[1];
    string config_file_path=argv[2];
    string output_path=argv[3];

    boost::filesystem::path src_cloud_path_bf(src_cloud_path);
    if (!boost::filesystem::exists(src_cloud_path_bf))
    {
        cout<<"error in finding src_cloud"<<endl;
        return 0;
    }
    boost::filesystem::path config_file_path_bf(config_file_path);
    if (!boost::filesystem::exists(config_file_path_bf))
    {
        cout<<"error in finding config_file"<<endl;
        return 0;
    }
    boost::filesystem::path output_path_bf(output_path);
    if (!boost::filesystem::exists(output_path_bf))
    {
        cout<<"error in fingding output_dir,create one"<<endl;
        if(!boost::filesystem::create_directory(output_path_bf))
        {
            cout<<"cannot create out_dir"<<endl;
            return 0;
        }
    }

    Config configs;
    FileIo::load_config_file(config_file_path,configs);

    CloudPtr cloud (new Cloud);
    NormalPtr normals (new NormalCloud);
    CloudPtr cloud_old (new Cloud);
    cout<<"load in point cloud: "<<src_cloud_path;
    pcl::io::loadPCDFile(src_cloud_path, *cloud_old);
    cout<<","<<cloud_old->size()<<"points contain"<<endl;
    PointFilter::voxel_sample(cloud_old,cloud,configs.voxel_scale);
    cout<<"voxel sample point cloud,scale:"<<configs.voxel_scale;
    cout<<","<<cloud->size()<<" points are lefted"<<endl;
    cout<<"start calculating feature..."<<endl;
    FeatureCalculate::rpca(cloud,normals,configs.num_of_neighbours,configs.pr,configs.epi,configs.relia);
/*    Visualizer::show_point_cloud_normal(cloud,normals,1,50,2);*/
    cout<<"start region growing..."<<endl;
    RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.initialize(cloud,normals,configs.smoothness_threshold,configs.curvature_threshold,configs.residual_threshold,configs.radius);
    vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    cout << "number of clusters is equal to " << clusters.size () << endl;
    vector<PlanSegment> planes;
    cout<<"start generating planes..."<<endl;
    reg.getPlaneFromPoint(planes,clusters);
    cout << "number of planes is equal to " << planes.size () << endl;
    Visualizer::show_plane_segment(planes,cloud,output_path);
    vector<LineSegment> lines;
    cout<<"start generating lines..."<<endl;
    LineGenerate::GenerateLineFromPlane(lines,planes,cloud);
    cout << "number of lines is equal to " << lines.size () << endl;
    cout<<"start refining lines..."<<endl;
    vector<LineSegment> refinedlines;
    LineGenerate::RefineLine(lines,refinedlines,configs);        /////////////填参数
    cout << "number of lines after refining is equal to " << refinedlines.size () << endl;
    Visualizer::show_line_segment(refinedlines,output_path);
    return 0;
}