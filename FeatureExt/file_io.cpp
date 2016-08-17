#include "StdAfx.h"
#include "file_io.h"

using std::string;
using std::endl;

FileIo::FileIo(void)
{
}


FileIo::~FileIo(void)
{
}

void FileIo::write_plane_file(const vector<PlanSegment>& planes,string plane_file_path)
{
    std::ofstream ofile(plane_file_path);
    for (int i=0;i<planes.size();i++)
    {
        ofile.setf(std::ios::fixed);
        ofile.precision(6);
        ofile<<i<<" "<<planes[i].normal.normal_x<<" "<<planes[i].normal.normal_y<<" "<<planes[i].normal.normal_z<<" "<<planes[i].distance<<std::endl;
    }
    ofile.close();
}
void FileIo::write_line_file(const vector<LineSegment>& lines,string out_path)
{
    stringstream mid,mif;
    mid <<"line"<< ".MID";
    mif <<"line"<< ".MIF";
    string mid_filename = mid.str();
    string mif_filename = mif.str();
    string mid_str = (boost::filesystem::path(out_path) / mid_filename).string();
    string mif_str = (boost::filesystem::path(out_path) / mif_filename).string();
    std::ofstream mid_ofile(mid_str);
    std::ofstream mif_ofile(mif_str);
    mif_ofile<<"Version   300"<<endl;
    mif_ofile<<"Charset \"WindowsSimpChinese\""<<endl;
    mif_ofile<<"Delimiter \",\""<<endl;
    mif_ofile<<"CoordSys Earth Projection 1, 104"<<endl;
    mif_ofile<<"Columns 1"<<endl;
    mif_ofile<<"  line_id Char(64)"<<endl;
    mif_ofile<<"Data"<<endl;
    mif_ofile<<endl;
    for (int i=0;i<lines.size();i++)
    {
        mid_ofile<<"\""<<i<<"\""<<endl;
        mif_ofile<<"Line "<<lines[i].startpt.x<<" "<<lines[i].startpt.y<<" "<<lines[i].endpt.x<<" "<<lines[i].endpt.y<<endl;
        mif_ofile<<"    Pen (1,2,0) "<<endl;
    }
    mid_ofile.close();
    mif_ofile.close();
}
void FileIo::load_config_file(const string config_file_path,Config &configs)
{
    boost::property_tree::ptree pt;  
    boost::property_tree::ini_parser::read_ini(config_file_path, pt);
    configs.voxel_scale=atof(pt.get<std::string>("VOXEL_SAMPLE.VOXEL_SCALE").c_str());

    configs.num_of_neighbours=atoi(pt.get<std::string>("RPCA.NUM_OF_NEIGHBOURS").c_str());
    configs.epi=atof(pt.get<std::string>("RPCA.EPI").c_str());
    configs.pr=atof(pt.get<std::string>("RPCA.PR").c_str());
    configs.relia=atof(pt.get<std::string>("RPCA.RELIA").c_str());

    configs.radius=atof(pt.get<std::string>("REGION_GROWING.RADIUS").c_str());
    configs.smoothness_threshold=atof(pt.get<std::string>("REGION_GROWING.SMOOTHNESS_THRESHOLD").c_str());
    configs.curvature_threshold=atof(pt.get<std::string>("REGION_GROWING.CURVATURE_THRESHOLD").c_str());
    configs.residual_threshold=atof(pt.get<std::string>("REGION_GROWING.RESIDUAL_THRESHOLD").c_str());
    configs.normalz_thresold=atof(pt.get<std::string>("REGION_GROWING.NORMALZ_THRESHOLD").c_str());
    configs.deltaz_threshold=atof(pt.get<std::string>("REGION_GROWING.DELTAZ_THRESHOLD").c_str());
    configs.cloud_size_threshold=atoi(pt.get<std::string>("REGION_GROWING.CLOUD_SIZE_THRESHOLD").c_str());

    configs.max_slope=atof(pt.get<std::string>("REFINE_LINE.MAX_SLOPE").c_str());
    configs.max_gap=atof(pt.get<std::string>("REFINE_LINE.MAX_GAP").c_str());
    configs.max_distance=atof(pt.get<std::string>("REFINE_LINE.MAX_DISTANCE").c_str());
    configs.max_length=atof(pt.get<std::string>("REFINE_LINE.MAX_LENGTH").c_str());
    configs.extension=atof(pt.get<std::string>("REFINE_LINE.EXTENSION").c_str());
}
