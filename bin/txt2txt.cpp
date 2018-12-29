/**
 * \brief 系统的输入数据与KITTI数据相同，imu/gps数据与相机/velodyne数据一一对应，
 * 一次测量数据单独保存为一个txt/bin文件. 
 * \func 本程序实现将一个txt转为KITTI的文件结构
 */

#include <iostream>
#include <fstream>

#include <pcl/console/parse.h>

#include <vector>
#include <string>

using namespace std;
using namespace pcl::console;

void
usage (char ** argv)
{
    cout << "usage: " << argv[0] << " [-original_txt <path-to-original-txt-file>] [-output_path <path-to-output-txt-file>] [-h | --help]" << endl;
    cout << argv[0] << " -h | --help : shows this help" << endl;
    return;
}

int
main (int argc,
      char ** argv)
{
    std::string original_txt, output_path;

    if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
    {
      usage (argv);
      return (0);
    }

    parse_argument (argc, argv, "-original_txt", original_txt);
    parse_argument (argc, argv, "-output_path", output_path);

    ifstream f;
    f.open(original_txt.c_str());

    // read header
    cout << "Header: \n" << endl;
    
    string s;
    getline(f,s);
    if(!s.empty())
    {
	string str1,str2,str3, str4;
	istringstream is(s);
	is >> str1 >> str2 >> str3 >> str4;
	cout << "  " << str1 << " " << str2 << " " << str3 << " " << str4 << endl;
    }
    else
    {
	std::cout << "Load failed!" << std::endl;
    }

    std::getline(f,s);
    if(!s.empty())
    {
	std::string str1,str2,str3, str4, str5;
	std::istringstream is(s);
	is >> str1 >> str2 >> str3 >> str4 >> str5;
	std::cout << "  " << str1 << " " << str2 << " " << str3 << " " << str4 << " " << str5 << std::endl;
    }
    else
    {
	std::cout << "Load failed!" << std::endl;
    }

    int count = 0;
    for(; count < 3; count++)
	getline(f, s);

    // output data
    count = 0;
    ofstream out_timestamps( "timestamps.txt", ios::out );
    while(!f.eof())
    {
        char tmp[256];
        std::sprintf(tmp, "%010d.bin", count);
        string out_filename(tmp);
	string out_full_path = output_path + out_filename;
        ofstream out_txt( out_full_path, ios::out );
	
	getline(f,s);
	if(!s.empty())
	{
	    string str1, str2, str3, str4, str5, str6, str7;
	    istringstream is(s);
	    is >> str1 >> str2 >> str3 >> str4 >> str5 >> str6 >> str7;
	    
	    out_timestamps << str1 << '\n';
	    
	    out_txt << std::to_string( atof(str2.c_str())*M_PI/180 ) << " " << 
	               std::to_string( atof(str3.c_str())*M_PI/180 ) << " " << 
	               std::to_string( atof(str4.c_str()) ) << " " << str5 << " " << str6 << " " << str7;
	}
	else
	{
	    std::cout << "Load failed!" << std::endl;
	}
	
	count++;
	out_txt.close();
    }
    
    out_timestamps.close();

    return (0);
}