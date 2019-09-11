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

#include "Initial.h"

namespace vhip_walking
{
  void states::Initial::start()
  {
    auto & ctl = controller();

    isWeighing_ = true;
    postureTaskIsActive_ = true;
    postureTaskWasActive_ = true;
    startStandingButton_ = false;
    startStanding_ = false;

    ctl.internalReset();

    logger().addLogEntry("walking_phase", []() { return -2.; });

    if (gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
    }

    runState(); // don't wait till next cycle to update reference and tasks
  }

  void states::Initial::teardown()
  {
    logger().removeLogEntry("walking_phase");

    if (gui())
    {
      hideStartStandingButton();
    }
  }

  void states::Initial::runState()
  {
    auto & ctl = controller();
    postureTaskIsActive_ = (ctl.postureTask->speed().norm() > 1e-2);
    if (postureTaskIsActive_)
    {
      hideStartStandingButton();
      postureTaskWasActive_ = true;
    }
    else if (postureTaskWasActive_)
    {
      ctl.internalReset();
      postureTaskWasActive_ = false;
    }
    else if (!isWeighing_)
    {
      showStartStandingButton();
    }
    weighRobot();
  }

  bool states::Initial::checkTransitions()
  {
    if (startStanding_ && !postureTaskIsActive_)
    {
      output("Standing");
      return true;
    }
    return false;
  }

  void states::Initial::showStartStandingButton()
  {
    if (!startStandingButton_ && gui())
    {
      using namespace mc_rtc::gui;
      gui()->addElement(
        {"Walking", "Controller"},
        Button(
          "Start standing",
          [this]() { startStanding_ = true; }));
      startStandingButton_ = true;
    }
  }

  void states::Initial::hideStartStandingButton()
  {
    if (startStandingButton_ && gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Start standing");
      startStandingButton_ = false;
    }
  }

  void states::Initial::weighRobot()
  {
    constexpr double MIN_GROUND_FORCE = 50.; // [N]
    constexpr double MAX_MASS_RELVAR = 0.2;

    auto & ctl = controller();
    double Fz = ctl.netWrenchObs().wrench().force().z();
    if (Fz < MIN_GROUND_FORCE)
    {
      return;
    }
    massEstimator_.add(Fz / world::GRAVITY);
    if (massEstimator_.n() < 100)
    {
      return;
    }

    double minRobotMass = (1. - MAX_MASS_RELVAR) * ctl.controlRobot().mass();
    double maxRobotMass = (1. + MAX_MASS_RELVAR) * ctl.controlRobot().mass();
    if (minRobotMass < massEstimator_.avg() && massEstimator_.avg() < maxRobotMass)
    {
      isWeighing_ = false;
      ctl.stabilizer().updateMass(massEstimator_.avg());
    }
    else
    {
      LOG_WARNING("Estimated mass " << massEstimator_.avg() << " [kg] "
          << "too far away from model mass " << ctl.controlRobot().mass() << " [kg]");
      isWeighing_ = true;
      hideStartStandingButton();
    }
  }
}

EXPORT_SINGLE_STATE("Initial", vhip_walking::states::Initial)
