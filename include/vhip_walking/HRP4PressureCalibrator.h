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

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/logging.h>

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
  struct HRP4PressureCalibrator
  {
    /** Default constructor.
     *
     */
    HRP4PressureCalibrator(double rate = 0.5)
      : rate_(rate)
    {
      reset();
    }

    /** Reset estimator.
     *
     */
    void reset()
    {
      nbSamples_ = 0;
      thetaSum_.setZero();
      theta_.setZero();
      x_ = {0., 0., 1.};
      y_ = 0.;
    }

    /** Add GUI tab.
     *
     * \param gui GUI handle.
     *
     */
    void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
    {
      using namespace mc_rtc::gui;
      gui->addElement(
        {"Calibrator"},
        Button(
          "Reset",
          [this]() { reset(); }),
        NumberInput(
          "Rate",
          [this]() { return rate(); },
          [this](double r) { rate(r); }),
        ArrayLabel(
          "Data",
          {"Fz", "Tx", "Ty"},
          [this]() -> Eigen::Vector3d { return {y_, x_[0], x_[1]}; }),
        Label(
          "Sample size",
          [this]() { return nbSamples_; }),
        Label(
          "Fz estimate",
          [this]() { return estimate(y_, x_[0], x_[1]); }));
    }

    /** Remove GUI tab.
     *
     * \param gui GUI handle.
     *
     */
    void removeGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
    {
      gui->removeCategory({"Calibrator"});
    }

    /** Log calibrator entries.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger)
    {
      logger.addLogEntry("calibrator_Fz_est", [this]() { return estimate(y_, x_[0], x_[1]); });
      logger.addLogEntry("calibrator_data_Fz", [this]() { return y_; });
      logger.addLogEntry("calibrator_data_Tx", [this]() { return x_[0]; });
      logger.addLogEntry("calibrator_data_Ty", [this]() { return x_[1]; });
      logger.addLogEntry("calibrator_internal_discount", [this]() { return std::pow(nbSamples_, -rate_); });
      logger.addLogEntry("calibrator_internal_nbSamples", [this]() { return nbSamples_; });
      logger.addLogEntry("calibrator_internal_residual", [this]() { return y_ - x_.dot(theta_); });
      logger.addLogEntry("calibrator_rate", [this]() { return rate_; });
      logger.addLogEntry("calibrator_theta", [this]() { return theta_; });
    }
  
    /** Remove log entries.
     *
     * \param logger Logger.
     *
     */
    void removeLogEntries(mc_rtc::Logger & logger)
    {
      logger.removeLogEntry("calibrator_Fz_est");
      logger.removeLogEntry("calibrator_data_Fz");
      logger.removeLogEntry("calibrator_data_Tx");
      logger.removeLogEntry("calibrator_data_Ty");
      logger.removeLogEntry("calibrator_theta");
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
    double estimate(double Fz, double Tx, double Ty)
    {
      Eigen::Vector3d thetaAvg = thetaSum_ / nbSamples_;
      return Fz - thetaAvg[0] * Tx - thetaAvg[1] * Ty;
    }
  
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
    void update(double Fz, double Tx, double Ty)
    {
      if (std::abs(x_[2] - 1.) > 1e-10 || rate_ <= 0. || rate_ >= 1.)
      {
        LOG_ERROR("HRP4PressureCalibrator parameters are wrong");
      }
      if (nbSamples_ == 0)
      {
        theta_[0] = 0.;
        theta_[1] = 0.;
        theta_[2] = Fz;
      }
      else // (nbSamples_ > 0)
      {
        double discount = std::pow(nbSamples_, -rate_);
        LOG_INFO("\n=======");
        LOG_INFO("theta = " << theta_.transpose());
        LOG_INFO("y = " << y_);
        LOG_INFO("x = " << x_.transpose());
        LOG_INFO("residual = " << (y_ - x_.dot(theta_)));
        LOG_INFO("dtheta = " << (discount * (y_ - x_.dot(theta_)) * x_));
        theta_ += discount * (y_ - x_.dot(theta_)) * x_;
      }
      y_ = Fz;
      x_[0] = Tx;
      x_[1] = Ty;
      //x_[2] = 1.;
      thetaSum_ += theta_;
      nbSamples_++;
    }
  
  private:
    Eigen::Vector3d thetaSum_;
    Eigen::Vector3d theta_; /**< Internal estimation parameters */
    Eigen::Vector3d x_; /**< Measured features {Tx, Ty, 1} */
    double rate_; /**< Learning rate */
    double y_; /**< Vector of observations Fz */
    unsigned nbSamples_; /**< Number of calls to update() */
  };
}
