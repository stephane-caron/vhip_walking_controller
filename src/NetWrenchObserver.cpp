/* 
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
    rawZMP_ = computeZMP(rawWrench_, contact);
    netZMP_ = computeZMP(netWrench_, contact);
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
  Eigen::Vector3d NetWrenchObserver::computeZMP(const sva::ForceVecd & wrench, const Contact & contact)
  {
    const Eigen::Vector3d & force = wrench.force();
    const Eigen::Vector3d & moment_0 = wrench.couple();
    Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);
    if (force.dot(force) > 42.) // force norm is more than 5 [N]
    {
      return contact.p() + contact.n().cross(moment_p) / contact.n().dot(force);
    }
    return Eigen::Vector3d::Zero();
  }

  void NetWrenchObserver::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("netWrench_forceCalib", [this]() { return forceCalib_; });
    logger.addLogEntry("netWrench_rawWrench", [this]() { return rawWrench_; });
    logger.addLogEntry("netWrench_rawZMP", [this]() { return rawZMP_; });
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
