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
    calibrator_.addLogEntries(logger());

    if (gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      calibrator_.addGUIElements(gui());
    }

    runState(); // don't wait till next cycle to update reference and tasks
  }

  void states::Initial::teardown()
  {
    logger().removeLogEntry("walking_phase");
    calibrator_.removeLogEntries(logger());

    if (gui())
    {
      hideStartStandingButton();
      calibrator_.removeGUIElements(gui());
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
    calibrateForceTorqueSensors();
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

  void states::Initial::calibrateForceTorqueSensors()
  {
    auto & ctl = controller();
    auto & robot = ctl.controlRobot();

    sva::PTransformd X_0_lf = robot.surface("LeftFoot").X_0_s(robot);
    sva::PTransformd X_0_rf = robot.surface("RightFoot").X_0_s(robot);
    sva::PTransformd X_0_mid = sva::interpolate(X_0_lf, X_0_rf, 0.5);

    sva::ForceVecd wrench_0 = ctl.netWrenchObs().rawWrench();
    sva::ForceVecd wrench_mid = X_0_mid.dualMul(wrench_0);
    double Fz = wrench_mid.force().z();
    double Tx = wrench_mid.moment().x();
    double Ty = wrench_mid.moment().y();
    calibrator_.update(Fz, Tx, Ty);
  }
}

EXPORT_SINGLE_STATE("Initial", vhip_walking::states::Initial)
