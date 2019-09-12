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

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <vhip_walking/Controller.h>
#include <vhip_walking/HRP4ForceCalibrator.h>
#include <vhip_walking/State.h>
#include <vhip_walking/utils/stats.h>

namespace vhip_walking
{
  /** Initial state.
   *
   * Check that contacts match foot positions.
   *
   */
  namespace states
  {
    struct Initial : State
    {
      /** Start state.
       *
       */
      void start() override;

      /** Teardown state.
       *
       */
      void teardown() override;

      /** Check transitions at beginning of control cycle.
       *
       */
      bool checkTransitions() override;

      /** Main state function, called if no transition at this cycle.
       *
       */
      void runState() override;

      /** Calibrate foot force torque sensors.
       *
       */
      void calibrateForceTorqueSensors();

      /** Remove "Start standing" transition button from GUI.
       *
       */
      void hideStartStandingButton();

      /** Add "Start standing" transition button to GUI.
       *
       */
      void showStartStandingButton();

      /** Estimate robot mass from force sensor measurements.
       *
       */
      void weighRobot();

    private:
      AvgStdEstimator massEstimator_;
      HRP4ForceCalibrator calibrator_;
      bool isWeighing_;
      bool postureTaskIsActive_;
      bool postureTaskWasActive_;
      bool startStandingButton_;
      bool startStanding_;
    };
  }
}
