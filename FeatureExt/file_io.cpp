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
