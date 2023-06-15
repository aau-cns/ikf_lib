/******************************************************************************
* FILENAME:     IO.hpp
* PURPOSE:      IO
* AUTHOR:       jungr - Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_UTILS_IO_HPP
#define IKF_UTILS_IO_HPP
#include <ikf/ikf_api.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <sys/types.h> // required for stat.h
#include <sys/stat.h> // no clue why required -- man pages say so
#include <fts.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <chrono>
#include <iostream>

namespace ikf
{
namespace utils
{
  template<class T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
  {
    os << "[";

    for(typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
      os << " " << *ii;
    }

    os << "]";
    return os;
  }

  class IKF_API IO
  {
  public:
    template<typename Map>
    static bool key_value_compare(Map const& lhs, Map const& rhs)
    {

      auto pred = [](decltype(*lhs.begin()) a, decltype(a) b)
      {
        return (a.first == b.first) && (a.second == b.second);
      };

      return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
    }



    template<typename T2, typename T1>
    static T2 lexical_cast(const T1& in)
    {
      T2                out;
      std::stringstream ss;
      ss << in;
      ss >> out;
      return out;
    }

    static std::string getFirstTokenAndReset(std::ifstream& os, std::string const delim = ",");

    typedef std::vector<std::string> vTokens;
    static void getTokens(vTokens& tokens, std::string const& str, std::string const delim);

    static std::vector<std::string> getTokens(std::string const& str);


    static std::string getFirstToken(std::string const& str, std::string const delim = ",");

    static std::vector<std::string> getTokens(std::string const& str, std::string const delim);

    static bool fileExists(const std::string& name);

    static bool dirExist(const std::string& dir);

    static bool isAbsolutePath(const std::string& path);

    static std::string getFileExtension(const std::string& filename);

    static std::string getFileDir(const std::string& filename);

    static std::string getFileName(const std::string& filename);

    static std::string getFileParentDirName(const std::string& filename);


    static bool createDirectory(const std::string& dirName);

    static bool createDirectoryRecursive(const std::string& dirName);

    static bool removeDirectory(const std::string& dirName);

    static void getFilesInDirectory(std::vector<std::string>& out,
                                           const std::string& directory,
                                           std::string extension = ""); // getFilesInDirectory

    static bool openFile(const std::string& filename, std::fstream& f);

    static bool createDirectoryFull(const std::string& baseDir);

    static std::vector<std::string> getLinesFromFile(std::string const& file_name);

    static void getAbsPathsFromFile(std::string const& list_file, vTokens& vTokens);

    static std::string checkPathString(std::string pathString);

  };





  // class IO
  } // namespace utilities
  } // namespace ikf

#endif // UTILS_IO_HPP
