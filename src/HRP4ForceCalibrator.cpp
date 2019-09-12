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

#include <vhip_walking/HRP4ForceCalibrator.h>

namespace vhip_walking
{
  HRP4ForceCalibrator::HRP4ForceCalibrator(double rate)
    : rate_(rate)
  {
    reset();
  }

  void HRP4ForceCalibrator::reset()
  {
    nbSamples_ = 0;
    thetaSum_.setZero();
    theta_.setZero();
    x_ = {0., 0., 1.};
    y_ = 0.;
  }

  void HRP4ForceCalibrator::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;
    gui->addElement(
      {"Calibrator"},
      Button(
        "Reset",
        [this]() { reset(); }),
      Checkbox(
        "Pause",
        [this]() { return paused_; },
        [this]() { paused_ = !paused_; }),
      NumberInput(
        "Rate",
        [this]() { return rate(); },
        [this](double r) { rate(r); }),
      ArrayLabel(
        "Data",
        {"Fz", "Tx", "Ty"},
        [this]() -> Eigen::Vector3d { return {y_, x_[0], x_[1]}; }),
      ArrayInput(
        "Theta",
        {"Tx", "Ty", "1"},
        [this]() { return thetaAvg(); },
        [this](const Eigen::Vector3d & theta)
        {
          thetaSum_ = nbSamples_ * theta;
          theta_ = theta;
        }),
      Label(
        "Fz estimate",
        [this]() { return estimate(y_, x_[0], x_[1]); }),
      Label(
        "Sample size",
        [this]() { return nbSamples_; }));
  }

  void HRP4ForceCalibrator::removeGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    gui->removeCategory({"Calibrator"});
  }

  void HRP4ForceCalibrator::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("calibrator_Fz_est", [this]() { return estimate(y_, x_[0], x_[1]); });
    logger.addLogEntry("calibrator_data_Fz", [this]() { return y_; });
    logger.addLogEntry("calibrator_data_Tx", [this]() { return x_[0]; });
    logger.addLogEntry("calibrator_data_Ty", [this]() { return x_[1]; });
    logger.addLogEntry("calibrator_theta", [this]() { return theta_; });
    logger.addLogEntry("calibrator_thetaAvg", [this]() { return thetaAvg(); });
  }

  void HRP4ForceCalibrator::removeLogEntries(mc_rtc::Logger & logger)
  {
    logger.removeLogEntry("calibrator_Fz_est");
    logger.removeLogEntry("calibrator_data_Fz");
    logger.removeLogEntry("calibrator_data_Tx");
    logger.removeLogEntry("calibrator_data_Ty");
    logger.removeLogEntry("calibrator_theta");
    logger.removeLogEntry("calibrator_thetaAvg");
  }

  void HRP4ForceCalibrator::update(double Fz, double Tx, double Ty)
  {
    if (std::abs(x_[2] - 1.) > 1e-10 || rate_ <= 0. || rate_ >= 1.)
    {
      LOG_ERROR("HRP4ForceCalibrator parameters are wrong");
    }
    if (nbSamples_ == 0)
    {
      theta_ = {0., 0., Fz};
    }
    else if (!paused_) // (nbSamples_ > 0)
    {
      double discount = std::pow(nbSamples_, -rate_);
      theta_ += discount * (y_ - x_.dot(theta_)) * x_;
      theta_.x() = std::min(5., std::max(-5., theta_.x()));
      theta_.y() = std::min(5., std::max(-5., theta_.y()));
      theta_.z() = std::min(500., std::max(100., theta_.z()));
    }
    y_ = Fz;
    x_ = {Tx, Ty, 1.};
    nbSamples_++;
    thetaSum_ += theta_;
  }
}
