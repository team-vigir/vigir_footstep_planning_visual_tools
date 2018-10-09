//=================================================================================================
// Copyright (c) 2018, Stephanie Ferreira, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#ifndef PATTERN_REQUEST_HANDLER_H
#define PATTERN_REQUEST_HANDLER_H

#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>

namespace vigir_footstep_planning_rviz_plugin
{
// Class to handle pattern planning requests
// Additionaly to the funcitonality of the base class pattern request handler can:
//  - send pattern request to a given pattern mode
//  - set parameters for pattern planning
//  - set current goal to position of last step (by using base request handlers functions)
//     (which is done always when a step plan is created)
class PatternRequestHandler : public RequestHandlerBase
{
  Q_OBJECT
public:
  PatternRequestHandler(QObject *parent = 0);
  virtual ~PatternRequestHandler();
  void sendPatternRequest(int patternMode, bool append);

public Q_SLOTS:
  //set Pattern Parameters:
  void setNoSteps(int no_steps);
  void setStepDistance(double step_dist);
  void setSideStep(double step_side);
  void setTurnAngle(int turn_angle);
  void setdz(double dz);

  //more Pattern Parameters
  void setStartStepIndex(int start_step);
  void setRoll(int roll);
  void setPitch(int pitch);

  //checkboxes:
  void setClosingStep(int state);
  void setExtraSeperation(int state);
  void setUseTerrainModel(int state);
  void setOverride3D(int state);

protected:
  void appendStepPlan(StepPlanMsg add) override;

private:
  void setPatternMode(int mode);
};
} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PATTERN_REQUEST_HANDLER_H
