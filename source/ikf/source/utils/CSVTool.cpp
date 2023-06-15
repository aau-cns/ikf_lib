/******************************************************************************
* FILENAME:     CSVTool.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     15.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/CSVTool.hpp>
#include <ikf/utils/IO.hpp>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

namespace ikf {
//namespace utils{

  bool CSVTool::read_csv(CsvDataType &csv_data, const std::string &file_path, char delim)
  {
    if (!ikf::utils::IO::fileExists(file_path))
    {
      std::cout << "[Warning] File " << file_path << " does not exist." << std::endl;
      return false;
    }

    std::ifstream  file_;
    file_.open(file_path);
    if(!file_.is_open() || file_.eof()) {
      std::cout << "read_csv(): Error: CSV file is empty or was not opened!" << std::endl;
      return false;
    }
    // Check for header
    const int first_value_row = check_for_header(file_);

    if (first_value_row < 1)
    {
      std::cout << "read_csv(): Error: No header in CSV file" << std::endl;
      return false;
    }

    HeaderMapType header_map = get_header(file_, first_value_row - 1);

    // Initialize CSV data type
    const int rows = get_rows(file_);

    // Set line counter to first valued row
    if (first_value_row > 0)
    {
      set_line_couter_of_file(file_, first_value_row);
    }

    csv_data.clear();
    for(auto it=header_map.begin(); it != header_map.end(); it++) {
      csv_data[it->second].resize(rows-1, 0.0);
    }


    // Read columns associated to header tokens
    std::string line;
    int line_counter = 0;
    int parsed_row_counter = first_value_row; // header already parsed.

    while (std::getline(file_, line))
    {

      std::stringstream row_stream(line);
      std::string token;
      int column_counter = 0;

      double item;
      while (std::getline(row_stream, token, delim))
      {
        if(column_counter >= (int)header_map.size()) {
          std::cout << "read_csv(): Warning: too many entries in row!" << std::endl;
          ++column_counter; // to indicate a corrupted row
          break;
        }

        std::istringstream is(token);
        is >> item;
        csv_data[header_map[column_counter]][line_counter]=(item);
        ++column_counter;
      }

      // check if row was corrupted, if so, overwrite current line with the next one
      if(column_counter != (int)header_map.size()) {
        std::cout << "read_csv(): Warning: corrupted row=" << parsed_row_counter << " will be skipped!" << std::endl;
      }
      else {
        line_counter++;
      }

      // increment parsed row counter
      parsed_row_counter++;
    }

    // shrink to the actual size
    for(auto it=header_map.begin(); it != header_map.end(); it++) {
      csv_data[it->second].resize(line_counter);
    }
    file_.close();
    return true;
  }

  bool CSVTool::write_csv(const CsvDataType &csv_data, const std::string &filename, char delim)
  {
    std::fstream file_;
    if(!utils::IO::openFile(filename, file_)) {
      std::cout << "[Warning] File " << filename << " could not be created/opend." << std::endl;
      return false;
    }

    std::vector<std::string> IDs;
    IDs.reserve(csv_data.size());
    for (auto const& elem : csv_data) {
      IDs.push_back(elem.first);
    }

    if(IDs.empty()) {
      std::cout << "[Warning] No data for " << filename << " ." << std::endl;
      return true;
    }

    // write header
    for(auto it = IDs.begin(); it != IDs.end(); it++) {
      if (it != --IDs.end()) {
        file_ << *it << delim;
      } else {
        file_ << *it << std::endl;
      }
    }


    size_t const num_rows = csv_data.at(IDs.at(0)).size();

    for(size_t row = 0; row < num_rows; row++) {
      std::ostringstream ss;
      ss.precision(16);
      for(auto it = IDs.begin(); it != IDs.end(); it++) {
        if (it != --IDs.end()) {
          ss << csv_data.at(*it)[row] << delim;
        } else {
          ss << csv_data.at(*it)[row];
        }
      }
      file_ << ss.str() << std::endl;
    }

    file_.close();

    return true;
  }

  CSVTool::HeaderMapType CSVTool::get_header(std::ifstream &file_, const int &row, char delim)
  {
    set_line_couter_of_file(file_, row);

    HeaderMapType header_data;

    int count = 0;
    std::string line;
    std::getline(file_, line);
    std::stringstream row_stream(line);
    std::string token;

    while (std::getline(row_stream, token, delim))
    {
      token.erase(remove_if(token.begin(), token.end(), isspace), token.end());

      header_data[count] = token;
      count++;
    }

    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return header_data;
  }

  int CSVTool::check_for_header(std::ifstream &file_)
  {
    int count = 0;
    std::string line;
    while (std::getline(file_, line))
    {
      if (std::isdigit(line[line.find_first_not_of(" \t")]))
      {
        break;
      }

      ++count;
    }

    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return count;
  }

  void CSVTool::set_line_couter_of_file(std::ifstream &file_, const int &line_number)
  {
    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);

    std::string line;
    for (int k = 0; k < line_number; k++)
    {
      std::getline(file_, line);
    }
  }

  int CSVTool::get_rows(std::ifstream &file_)
  {
    int count = 0;
    std::string line;
    while (std::getline(file_, line))
    {
      ++count;
    }
    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return count;
  }
//} // ns utils
} // ns ikf
