/******************************************************************************
* FILENAME:     ModularInstanceHandler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef MODULARINSTANCEHANDLER_HPP
#define MODULARINSTANCEHANDLER_HPP
#include "mmsf/Model/SensorSuite.hpp"
#include <ikf/Strategies/IInstanceHandler.hpp>
namespace ikf {

class ModularInstanceHandler : public IInstanceHandler {
  ModularInstanceHandler(double const horizon_sec=1.0): HistMeas(horizon_sec), m_horzion_sec(horizon_sec) {

  }

  void initialize(SensorSuite const& suite, Timestamp const& t_0) {

  }
  // IInstanceHandler interface
public:
  ProcessMeasResult_t process_measurement(const MeasData &m) {
    ProcessMeasResult_t res;

    if (exists(m.id_sensor)) {
      ptr_instance p_instance = get_by_id(m.id_sensor);
      if (p_instance->enabled()) {
        res = p_instance->process_measurement(m);
      } else {
        res.skipped = true;
      }
      if (!res.skipped && !res.rejected && HistMeas.size()) {
        Timestamp t_oldest, t_latest;
        HistMeas.get_oldest_t(t_oldest);
        HistMeas.get_latest_t(t_latest);

        if(m.t_m < t_oldest) {
          std::cout << "Sensor measurement for " << p_instance->get_Name() << ":Too much delay" << std::endl;
        }
        else if(m.t_m > t_latest) {
          redo_updates_after_t(m.t_m);
        }
        HistMeas.insert(m, m.t_m);
      }
      else if(!res.skipped) {
        HistMeas.insert(m, m.t_m);
      }
    }
    return res;
  }

  void redo_updates_after_t(Timestamp const& t) {
    remove_after_t(t);
    Timestamp t_latest;
    if(HistMeas.get_latest_t(t_latest)) {

      HistMeas.foreach_between_t1_t2(t, t_latest, [this](MeasData const& m) {
        ptr_instance p_inst = this->id_dict.at(m.id_sensor);
        p_inst->process_measurement(m);
      });
    }
  }


  void remove_after_t(Timestamp const& t) {
    for (auto const& elem : this->name_dict) {
      elem.second->get_MMSF_hdl()->remove_beliefs_after_t(t);
    }
  }

  void disp_measurement_hist()
  {
    HistMeas.foreach([](MeasData const& m) {
      std::cout << m << std::endl;
    });
  }

protected:
  TTimeHorizonBuffer<MeasData> HistMeas;
  double m_horzion_sec;
};

} // namespace ikf
#endif // MODULARINSTANCEHANDLER_HPP
