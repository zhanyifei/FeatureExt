#pragma once
#include "stdafx.h"

using std::string;
using std::vector;

class FileIo
{
public:
    FileIo(void);
    ~FileIo(void);
    static void write_plane_file(const vector<PlanSegment>& planes,string plane_file_path);
    static void write_line_file(const vector<LineSegment>& lines,string line_file_path);
};

