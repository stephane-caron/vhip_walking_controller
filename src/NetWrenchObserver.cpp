/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of vhip_walking_controller.
 *
 * vhip_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vhip_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with vhip_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <vhip_walking/NetWrenchObserver.h>

namespace vhip_walking
{
  NetWrenchObserver::NetWrenchObserver()
    : sensorNames_({"LeftFootForceSensor", "RightFootForceSensor"})
  {
  }

  NetWrenchObserver::NetWrenchObserver(const std::vector<std::string> & sensorNames)
    : sensorNames_(sensorNames)
  {
  }

  void NetWrenchObserver::update(const mc_rbdyn::Robot & robot, const Contact & contact)
  {
    updateNetWrench(robot);
    updateNetZMP(contact);
  }

  void NetWrenchObserver::updateNetWrench(const mc_rbdyn::Robot & robot)
  {
    netWrench_ = sva::ForceVecd::Zero();
    for (std::string sensorName : sensorNames_)
    {
      const auto & sensor = robot.forceSensor(sensorName);
      if (sensor.force().z() > 1.) // pressure is more than 1 [N]
      {
        netWrench_ += sensor.worldWrench(robot);
      }
    }
  }

  void NetWrenchObserver::updateNetZMP(const Contact & contact)
  {
    const Eigen::Vector3d & force = netWrench_.force();
    const Eigen::Vector3d & moment_0 = netWrench_.couple();
    Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);
    if (force.dot(force) > 42.) // force norm is more than 5 [N]
    {
      netZMP_ = contact.p() + contact.n().cross(moment_p) / contact.n().dot(force);
    }
  }

  void NetWrenchObserver::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("netWrench_wrench", [this]() { return netWrench_; });
    logger.addLogEntry("netWrench_zmp", [this]() { return netZMP_; });
  }
}
