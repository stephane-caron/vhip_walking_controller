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

#include <memory>

#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/GUIState.h>
#include <mc_rtc/log/Logger.h>

namespace vhip_walking
{
  /** Calibration routine due to an unmodeled coupling between ground reaction
   * force and torque measurements on HRP-4.
   *
   * This estimator is based on Polyak averaging, see
   * <https://stats.stackexchange.com/a/81783/259718>.
   *
   * The notations use here are \f$y = F_z\f$ and \f$x = (T_x, T_y, 1)\f$ so
   * that:
   *
   * \f$ F_z = \theta_0 T_x + \theta_1 T_y + \theta_2\f$
   *
   * Measurements are supposed to be conducted with the robot standing still on
   * the ground, with the user pushing it horizontally but not vertically. This
   * way, \f$\theta_2\f$ converges to the actual (constant) vertical force, while
   * \f$(\theta_0, \theta_1)\f$ capture linear correlations in \f$(F_z, T_x,
   * T_y)\f$.
   *
   */
  struct HRP4ForceCalibrator
  {
    /** Default constructor.
     *
     * \param rate Learning rate.
     *
     */
    HRP4ForceCalibrator(double rate = 0.5);

    /** Reset estimator.
     *
     */
    void reset();

    /** Add GUI tab.
     *
     * \param gui GUI handle.
     *
     */
    void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

    /** Remove GUI tab.
     *
     * \param gui GUI handle.
     *
     */
    void removeGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

    /** Log calibrator entries.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger);

    /** Remove log entries.
     *
     * \param logger Logger.
     *
     */
    void removeLogEntries(mc_rtc::Logger & logger);

    /** Update estimation from new measurements. These measurements are supposed
     * to be conducted with the robot standing still on the ground, with the user
     * pushing it horizontally but not vertically.
     *
     * \param Fz Measured net reaction force.
     *
     * \param Tx Measured net reaction torque around the sagittal axis.
     *
     * \param Ty Measured net reaction torque around the lateral axis.
     *
     */
    void update(double Fz, double Tx, double Ty);

    /** Estimate vertical net force from new measurements.
     *
     * \param Fz Vertical net reaction force estimated from foot F/T sensors.
     *
     * \param Tx Net reaction torque around the sagittal axis.
     *
     * \param Ty Net reaction torque around the lateral axis.
     *
     * \returns Fz_est Estimated vertical reaction force on the actual ground.
     *
     */
    double estimate(double Fz, double Tx, double Ty) const
    {
      return Fz - thetaAvg().dot(Eigen::Vector3d{Tx, Ty, 0.});
    }

    /** Learning rate of the internal estimator.
     *
     */
    double rate() const
    {
      return rate_;
    }

    /** Update learning rate of the internal estimator.
     *
     * \param rate New learning rate between 0.5 and 1.
     *
     */
    void rate(double rate)
    {
      rate_ = std::min(0.95, std::max(0.05, rate));
    }

    /** Polyak average of fluctuating parameter estimates.
     *
     */
    Eigen::Vector3d thetaAvg() const
    {
      return thetaSum_ / nbSamples_;
    }

  private:
    Eigen::Vector3d thetaSum_; /**< Sum of fluctuating parameter estimates */
    Eigen::Vector3d theta_; /**< Internal estimation parameters */
    Eigen::Vector3d x_; /**< Measured features {Tx, Ty, 1} */
    bool paused_ = true;
    double rate_; /**< Learning rate */
    double y_; /**< Vector of observations Fz */
    unsigned nbSamples_; /**< Number of calls to update() */
  };
}
