/******************************************************************************
* FILENAME:     IInstanceHandler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     31.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IINSTANCEHANDLER_HPP
#define IINSTANCEHANDLER_HPP
#include <ikf/ikf_api.h>
#include <ikf/Strategies/IInstanceHandle.hpp>

namespace ikf {


class IKF_API IInstanceHandler {
public:
  virtual ~IInstanceHandler() {}


  bool add(std::shared_ptr<IInstanceHandle> ptr_instance);
  bool remove(size_t const ID);
  bool remove(std::string const& name);
  bool exists(size_t const ID);
  bool exists(std::string const& name);
  std::vector<size_t> get_instance_ids();
  std::vector<std::string> get_instance_names();
  std::shared_ptr<IInstanceHandle> get_by_id(size_t const ID);
  std::shared_ptr<IInstanceHandle> get_by_name(std::string const name);
  virtual std::shared_ptr<IMMSF> get_IKF_hdl(const size_t ID);
  void reset();

  virtual ProcessMeasResult_t process_measurement(MeasData const& m) = 0;

  // for CSE
  // virtual bool redo_updates_after_t(std::vector<size_t> const& ID_j, Timestamp const& t) = 0;
protected:
  std::unordered_map<size_t, std::shared_ptr<IInstanceHandle>> id_dict;
  std::unordered_map<std::string, std::shared_ptr<IInstanceHandle>> name_dict;

};



// class IInstanceHandler

} // namespace ikf
#endif // IINSTANCEHANDLER_HPP
