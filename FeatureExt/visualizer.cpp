#include "StdAfx.h"
#include "visualizer.h"

using std::string;


Visualizer::Visualizer(void)
{
}


Visualizer::~Visualizer(void)
{
}

void Visualizer::show_point_cloud_normal(CloudPtr cloud,NormalPtr normal,int point_size,int show_level,int normal_size)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Normal Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normal,show_level,normal_size,"normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
void Visualizer::show_point_cloud_curvature(CloudPtr cloud,NormalPtr normal,int point_size)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Curvature Viewer"));
    viewer->setBackgroundColor(255,255,255);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int i=0;i<cloud->size();i++)
    {
        pcl::PointXYZRGBA tempoint;
        tempoint.x=cloud->points[i].x;
        tempoint.y=cloud->points[i].y;
        tempoint.z=cloud->points[i].z;
         uint8_t _r,_g,_b;
        _r=normal->points[i].curvature/0.1f*255.0f;
        _g=normal->points[i].curvature/0.1f*255.0f;
        _b=normal->points[i].curvature/0.1f*255.0f;
        tempoint.r=_r;
        tempoint.g=_g;
        tempoint.b=_b;
        color_cloud->push_back(tempoint);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(color_cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (color_cloud,rgb,"curvature cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "curvature cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
}
void Visualizer::show_line_segment(const vector<LineSegment>& lines,string out_path)
{
    srand((unsigned)time(NULL));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("LineSegmentDisplay"));
    viewer->setBackgroundColor(0,0,0);
    for (int i=0;i<lines.size();i++)
    {
        char temp[64];
        sprintf(temp, "%d", i);
        string linename=temp;
        viewer->addLine<pcl::PointXYZ>(lines[i].startpt,lines[i].endpt,temp);
    }
    FileIo::write_line_file(lines,out_path);
//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce();
//         //boost::this_thread::sleep (boost::posix_time::microseconds (1000));
//     }

}
void Visualizer::show_plane_segment(vector<PlanSegment>& planes,CloudPtr cloud,string out_path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    if (!planes.empty ())
    {
        colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

        srand (static_cast<unsigned int> (time (0)));
        std::vector<unsigned char> colors;
        for (size_t i_segment = 0; i_segment <  planes.size (); i_segment++)
        {
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
        }
        std::vector<PlanSegment >::iterator i_segment;
        int next_color = 0;
        for (i_segment = planes.begin (); i_segment !=  planes.end (); i_segment++)
        {
            std::vector<int>::iterator i_point;
            for (i_point = i_segment->points_id.indices.begin (); i_point != i_segment->points_id.indices.end (); i_point++)
            {
                int index;
                index = *i_point;
                pcl::PointXYZRGB color_point;
                color_point.x=cloud->points[index].x;
                color_point.y=cloud->points[index].y;
                color_point.z=cloud->points[index].z;
                color_point.r= colors[3 * next_color];
                color_point.g= colors[3 * next_color + 1];
                color_point.b= colors[3 * next_color + 2];
                colored_cloud->push_back(color_point);
            }
            next_color++;
        }
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Plane Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud,rgb,"plane cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
    stringstream out;
    out <<"plane"<< ".txt";
    string filename = out.str();
    string str = (boost::filesystem::path(out_path) / filename).string();
    FileIo::write_plane_file(planes,str);
    stringstream pcd_out;
    pcd_out<<"colored.pcd";
    filename=pcd_out.str();
    str=(boost::filesystem::path(out_path) / filename).string();
    pcl::io::savePCDFileBinary(str, *colored_cloud);
//     while (!viewer->wasStopped())
//     {
//         viewer->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (1000));
//     }
}
