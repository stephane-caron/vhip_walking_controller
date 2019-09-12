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

  void NetWrenchObserver::update(const mc_rbdyn::Robot & realRobot, const Contact & contact)
  {
    updateNetWrench(realRobot);
    updateNetZMP(contact);
  }

  void NetWrenchObserver::updateNetWrench(const mc_rbdyn::Robot & realRobot)
  {
    rawWrench_ = sva::ForceVecd::Zero();
    for (std::string sensorName : sensorNames_)
    {
      const auto & sensor = realRobot.forceSensor(sensorName);
      if (sensor.force().z() > 1.) // pressure is more than 1 [N]
      {
        // map sensor wrench to inertial frame using realRobot kinematics
        rawWrench_ += sensor.worldWrench(realRobot);
      }
    }

    const sva::PTransformd & X_0_anchor = anchorFrame_;
    sva::ForceVecd anchorWrench = X_0_anchor.dualMul(rawWrench_);
    const Eigen::Vector3d & force = anchorWrench.force();
    const Eigen::Vector3d & moment = anchorWrench.moment();
    double Fz = force.z() - forceCalib_.dot(moment.head<2>());
    sva::ForceVecd corrAnchorWrench = {moment, {force.x(), force.y(), Fz}};
    netWrench_ = X_0_anchor.inv().dualMul(corrAnchorWrench);
  }

  /* TODO: update w.r.t. anchorFrame
   *
   */
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
    logger.addLogEntry("netWrench_rawWrench", [this]() { return rawWrench_; });
    logger.addLogEntry("netWrench_wrench", [this]() { return netWrench_; });
    logger.addLogEntry("netWrench_zmp", [this]() { return netZMP_; });
  }

  /* TODO: merge with Stabilizer's
   *
   */
  void NetWrenchObserver::updateAnchorFrame(ContactState contactState, const mc_rbdyn::Robot & controlRobot)
  {
    sva::PTransformd X_0_lf = controlRobot.surface("LeftFoot").X_0_s(controlRobot);
    sva::PTransformd X_0_rf = controlRobot.surface("RightFoot").X_0_s(controlRobot);
    if (contactState == ContactState::DoubleSupport)
    {
      anchorFrame_ = sva::interpolate(X_0_lf, X_0_rf, 0.5);
    }
    else if (contactState == ContactState::LeftFoot)
    {
      anchorFrame_ = X_0_lf;
    }
    else // (contactState == ContactState::RightFoot)
    {
      anchorFrame_ = X_0_rf;
    }
  }
}
