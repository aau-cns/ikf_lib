/******************************************************************************
* FILENAME:     CSVTool.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     15.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_CSVTOOL_HPP
#define IKF_CSVTOOL_HPP
#include <fstream>
#include <ikf/ikf_api.h>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace ikf {

// Adapted from: https://github.com/aau-cns/mars_lib/blob/main/source/mars/include/mars/data_utils/read_csv.h
class IKF_API CSVTool
{
public:
 typedef std::map<std::string, double> CsvRowDataType;
 typedef std::map<std::string, std::vector<double>> CsvDataType;
 typedef std::unordered_map<std::string, std::vector<double>> CsvDataUnorderedType;

 typedef std::map<int, std::string> HeaderMapType;

 static bool read_csv(CsvDataType& csv_data, const std::string& file_path, char delim = ',');

 static bool write_csv(CsvRowDataType const& csv_data, const std::string& filename, char delim = ',');
 static bool write_csv(CsvDataType const& csv_data, const std::string& filename, char delim = ',');
 static bool write_csv(CsvDataUnorderedType const& csv_data, const std::string& filename, char delim = ',');

 static HeaderMapType get_header(std::ifstream& file_, const int& row = 0, char delim = ',');

 static int check_for_header(std::ifstream& file_);

 static void set_line_couter_of_file(std::ifstream& file_, const int& line_number);

 static int get_rows(std::ifstream& file_);
};


} // ikfs

#endif // CSVTOOL_HPP
