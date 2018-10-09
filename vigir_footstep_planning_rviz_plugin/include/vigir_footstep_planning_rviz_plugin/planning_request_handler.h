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


#ifndef PLANNING_REQUEST_HANDLER_H
#define PLANNING_REQUEST_HANDLER_H

#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>


namespace vigir_footstep_planning_rviz_plugin
{
// Class to handle planning requests
// Additionaly to the funcitonality of the base class planning request handler can:
//  - receive a request to request the correct goal pose for a given position and emit the resulting current goal pose
//  - send a planning request
class PlanningRequestHandler : public RequestHandlerBase
{
  Q_OBJECT
public:
  PlanningRequestHandler(QObject *parent = 0);
  virtual ~PlanningRequestHandler();

  void sendPlanningRequest(bool append);
 // void setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void setGoal(vigir_footstep_planning_msgs::Feet goal_feet);
  void replanToIndex(int index);

public Q_SLOTS:
  void setStartStepIndex(int start_step);
  void setMaxPlanningTime(double t);
  void setMaxNofSteps(int noSteps);
  void setMaxPathLengthRatio(double ratio);
  void setActivateOnPlaceFeet(bool activate); //start step plan generation when feet are placed
  void setAppend(bool appending);

protected:
  void appendStepPlan(StepPlanMsg add) override;

private:
  void replanToIndexCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

  void goalPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

  bool activate_on_place_feet;
  bool append;
};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PLANNING_REQUEST_HANDLER_H
