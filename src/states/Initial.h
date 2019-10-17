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
