/******************************************************************************
* FILENAME:     IO.cpp
* PURPOSE:      IO
* AUTHOR:       jungr - Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/IO.hpp>

namespace ikf
{
namespace utils
{

  std::vector<std::string> IO::getLinesFromFile(const std::string& file_name)
  {
    std::vector<std::string> vgmf_files;

    std::ifstream file(file_name);

    if(file.is_open())
    {
      std::string s;

      while(std::getline(file, s))
      {
        vgmf_files.push_back(s);
      }
    }
    else
    {
      std::cout << "ikf::utils::IO: unable to open file: " << file_name;
    }

    return vgmf_files;
  }

  void IO::getAbsPathsFromFile(const std::string& list_file, IO::vTokens &vTokens)
  {
    if(!list_file.empty())
    {
      IO::vTokens v        = IO::getLinesFromFile(list_file);
      std::string base_dir = IO::getFileDir(list_file);

      for(std::string& x : v)
      {
        if(!IO::isAbsolutePath(x))
        {
          x = std::string(base_dir + "/" + x);
        }

        vTokens.push_back(x);
      }
    }
  }

  std::string IO::checkPathString(std::string pathString)
  {
    // execute the string replacement as long as there is a double slash in the string
    for(; ;)
    {
      int pos = pathString.find("//");

      if(pos < 0)
      {
        break;
      }

      pathString.replace(pos, 2, "/");
    }

    return pathString;
  }


  std::string IO::getFirstTokenAndReset(std::ifstream &os, const std::string delim)
  {
    int cur_pos = os.tellg();

    if(cur_pos < 0)
    {
      return std::string();
    }

    std::string t;
    os >> t;
    os.seekg(cur_pos);
    return t;
  }

  void IO::getTokens(vTokens &tokens, const std::string &str, const std::string delim)
  {
    size_t prev = 0, pos = 0;
    tokens.clear();

    do
    {
      pos = str.find(delim, prev);

      if(pos == std::string::npos || (pos == 0 && prev != 0))
      {
        pos = str.length();
      }

      std::string token = str.substr(prev, pos - prev);

      if(!token.empty())
      {
        tokens.emplace_back(std::move(token));
      }

      prev = pos + delim.length();
    }
    while(pos < str.length() && prev < str.length());
  }

  std::vector<std::string> IO::getTokens(const std::string &str)
  {
    std::stringstream        ss(str);
    std::vector<std::string> Tokens;
    std::string              Temp;

    while(ss >> Temp)
    {
      Tokens.push_back(Temp);
    }

    return Tokens;
  }

  std::string IO::getFirstToken(const std::string &str, const std::string delim)
  {
    size_t prev = 0, pos = 0;
    pos = str.find(delim, prev);

    if(pos == std::string::npos || (pos == 0 && prev != 0))
    {
      return str;
    }

    return str.substr(prev, pos - prev);

  }

  std::vector<std::string> IO::getTokens(const std::string &str, const std::string delim)
  {
    std::vector<std::string> tokens;
    getTokens(tokens, str, delim);
    return tokens;
  }

  bool IO::fileExists(const std::string &name)
  {
    std::ifstream f(name.c_str());
    return f.good();
  }

  bool IO::dirExist(const std::string &dir)
  {
    DIR *d = opendir(dir.c_str());

    if(d != NULL)
    {
      closedir(d);
      return true;
    }

    return false;
  }

  bool IO::isAbsolutePath(const std::string &path)
  {
    return (path.data()[0] == '/');
  }

  std::string IO::getFileExtension(const std::string &filename)
  {
    std::string::size_type idx;

    idx = filename.rfind('.');

    if(idx != std::string::npos)
    {
      return std::string(filename.substr(idx + 1));
    }
    else
    {
      return std::string();
    }
  }

  std::string IO::getFileDir(const std::string &filename)
  {
    std::string::size_type idx;

    idx = filename.rfind('/');

    if(idx != std::string::npos)
    {
      return std::string(filename.substr(0, idx));
    }
    else
    {
      return std::string();
    }
  }

  std::string IO::getFileName(const std::string &filename)
  {
    std::string::size_type idx1, idx2;

    // TODO: this works only for absolute paths

    idx1 = filename.rfind('/');
    idx2 = filename.rfind('.');

    if(idx1 == std::string::npos && idx2 != std::string::npos)
    {
      // no absolute path
      return std::string(filename.substr(0, idx2));
    }
    else if(idx1 != std::string::npos && idx2 != std::string::npos)
    {
      int len = (idx2) - (idx1 + 1);
      return std::string(filename.substr(idx1 + 1, len));
    }
    else
    {
      return std::string();
    }
  }

  std::string IO::getFileParentDirName(const std::string &filename)
  {
    std::string            file_dir = getFileDir(filename);
    std::string::size_type idx      = file_dir.rfind('/');

    if(idx != std::string::npos)
    {
      return std::string(file_dir.substr(idx + 1, file_dir.length()));
    }
    else
    {
      return std::string();
    }
  }

  std::string IO::getLastDirName(const std::string &path)
  {
    std::string file_dir(path);
    std::string::size_type idx      = file_dir.rfind('/');

    if(idx != std::string::npos)
    {
      return std::string(file_dir.substr(idx + 1, file_dir.length()));
    }
    else
    {
      return std::string();
    }
  }

  bool IO::createDirectory(const std::string &dirName)
  {
    std::string sPath  = dirName;
    mode_t      nMode  = 0733; // UNIX style permissions
    int         nError = 0;
#if defined(_WIN32)
    nError = _mkdir(sPath.c_str()); // can be used on Windows
#else
    nError = mkdir(sPath.c_str(), nMode); // can be used on non-Windows
#endif

    if(nError != 0)
    {
      // handle your error here
      return false;
    }

    return true;
  }

  bool IO::createDirectoryRecursive(const std::string &dirName)
  {
    std::vector<std::string> vTokens = getTokens(dirName, "/");

    if(vTokens.size() < 1)
    {
      return false;
    }

    std::string path;

    if(isAbsolutePath(dirName)) // check if it is absolute
    {
      path = "/";
    }

    for(std::string& node : vTokens)
    {
      if(node != "")
      {

        path += node;

        if(!fileExists(path))
        {
          std::cout << "create " << path << std::endl;

          if(!createDirectory(path))
          {
            return false;
          }
        }

        path += "/";
      }
    }

    return true;
  }

  bool IO::removeDirectory(const std::string &dirName)
  {
    int res = system(std::string("rm -r " + dirName).c_str());
    return (res == 0);
  }

  void IO::getFilesInDirectory(std::vector<std::string> &out, const std::string &directory, std::string extension)
  {
    out.clear();

    if(!dirExist(directory))
    {
      return;
    }

    DIR           *dir;
    struct dirent *ent;
    struct stat   st;

    dir = opendir(directory.c_str());

    bool bCheckExtension = true;

    if(extension.empty())
    {
      bCheckExtension = false;
    }

    while((ent = readdir(dir)) != NULL)
    {
      const std::string file_name      = ent->d_name;
      const std::string full_file_name = directory + "/" + file_name;

      if(file_name[0] == '.')
      {
        continue;
      }

      if(stat(full_file_name.c_str(), &st) == -1)
      {
        continue;
      }

      const bool is_directory = (st.st_mode & S_IFDIR) != 0;

      if(is_directory)
      {
        continue;
      }

      if(bCheckExtension)
      {
        if(getFileExtension(file_name) != extension)
        {
          continue;
        }
      }

      out.push_back(full_file_name);
    }

    closedir(dir);
  }

  void ikf::utils::IO::getDirsInDirectory(std::vector<std::string> &out, const std::string &directory) {
    out.clear();

    if(!dirExist(directory))
    {
      return;
    }

    DIR           *dir;
    struct dirent *ent;
    struct stat   st;

    dir = opendir(directory.c_str());


    while((ent = readdir(dir)) != NULL)
    {
      const std::string file_name      = ent->d_name;
      const std::string full_file_name = directory + "/" + file_name;

      if(file_name[0] == '.')
      {
        continue;
      }

      if(stat(full_file_name.c_str(), &st) == -1)
      {
        continue;
      }

      const bool is_directory = (st.st_mode & S_IFDIR) != 0;

      if(is_directory)

      {
        out.push_back(full_file_name);
      }

    }

    closedir(dir);
  }

  bool IO::openFile(const std::string &filename, std::fstream &f)
  {

    if(!fileExists(filename))
    {
      std::string baseDir = getFileDir(filename);

      if(baseDir.empty())
      {
        return false;
      }

      if(!dirExist(baseDir))
      {
        createDirectoryRecursive(baseDir);

        if(!dirExist(baseDir))
        {
          std::cerr << "ikf::utils::IO: something went wrong creating dir: " << baseDir << std::endl;
          return false;
        }
      }
    }

    f.open(filename.c_str(), std::ios_base::out);

    if(!f.is_open())
    {
      return false;
    }

    return true;
  }

  bool IO::createDirectoryFull(const std::string &baseDir)
  {
    if(!dirExist(baseDir))
    {
      createDirectoryRecursive(baseDir);

      if(!dirExist(baseDir))
      {
        std::cerr << "ikf::utils::IO: something went wrong creating dir: " << baseDir << std::endl;
        return false;
      }
    }

    return true;
  }

} // ns utilities
} // ns ikf
