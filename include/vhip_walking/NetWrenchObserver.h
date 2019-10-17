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

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <vhip_walking/Contact.h>
#include <vhip_walking/Pendulum.h>
#include <vhip_walking/defs.h>
#include <vhip_walking/utils/clamp.h>

namespace vhip_walking
{
  /** Measure net contact wrench in the world frame from force/torque sensors.
   *
   */
  struct NetWrenchObserver
  {
    /** Empty constructor.
     *
     */
    NetWrenchObserver();

    /** Constructor from list of sensor names.
     *
     * \param sensorNames Identifiers of end-effector force-torque sensors.
     *
     */
    NetWrenchObserver(const std::vector<std::string> & sensorNames);

    /** Log stabilizer entries.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger);

    /** Update estimates based on the sensed net contact wrench.
     *
     * \param realRobot Kinematic observer for actual robot.
     *
     * \param contact Support contact frame.
     *
     */
    void update(const mc_rbdyn::Robot & realRobot, const Contact & contact);

    /** TODO: to be merged with Stabilizer's ZMP frame.
     *
     */
    void updateAnchorFrame(ContactState contactState, const mc_rbdyn::Robot & controlRobot);

    /** Get force calibration data.
     *
     */
    const Eigen::Vector2d & forceCalib() const
    {
      return forceCalib_;
    }

    /** Set force calibration data.
     *
     */
    void forceCalib(const Eigen::Vector2d & calib)
    {
      forceCalib_ = calib;
      clampInPlace(forceCalib_.x(), -2., +2., "Normal force calib KTx");
      clampInPlace(forceCalib_.y(), -5., +5., "Normal force calib KTy");
    }

    /** Raw contact wrench in the world frame before applying calibration data.
     *
     */
    const sva::ForceVecd & rawWrench() const
    {
      return rawWrench_;
    }

    /** Net contact wrench in the world frame.
     *
     */
    const sva::ForceVecd & wrench() const
    {
      return netWrench_;
    }

    /** Zero-tilting moment point in the latest contact frame.
     *
     */
    const Eigen::Vector3d & zmp() const
    {
      return netZMP_;
    }

  private:
    /** Update net wrench estimate from robot sensors.
     *
     * \param robot Robot state.
     *
     */
    void updateNetWrench(const mc_rbdyn::Robot & robot);

    /** Compute ZMP of a wrench in a given frame.
     *
     * \param wrench Wrench.
     *
     * \param contact Frame that defines the ZMP plane.
     *
     */
    Eigen::Vector3d computeZMP(const sva::ForceVecd & wrench, const Contact & contact);

  private:
    Eigen::Vector2d forceCalib_ = Eigen::Vector2d::Zero();
    Eigen::Vector3d netZMP_ = Eigen::Vector3d::Zero(); /**< Net wrench ZMP in the contact frame */
    Eigen::Vector3d rawZMP_ = Eigen::Vector3d::Zero(); /**< Net wrench ZMP in the contact frame */
    std::vector<std::string> sensorNames_ = {"LeftFootForceSensor", "RightFootForceSensor"}; /**< List of force/torque sensor identifiers */
    sva::ForceVecd netWrench_ = sva::ForceVecd::Zero(); /**< Corrected contact wrench in the world frame */
    sva::ForceVecd rawWrench_ = sva::ForceVecd::Zero(); /**< Raw contact wrench in the world frame */
    sva::PTransformd anchorFrame_ = sva::PTransformd::Identity();
  };
}
