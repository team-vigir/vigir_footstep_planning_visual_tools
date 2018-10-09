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


#ifndef REQUEST_HANDLER_BASE_H
#define REQUEST_HANDLER_BASE_H

#ifndef Q_MOC_RUN

#include <QObject>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <actionlib/client/simple_action_client.h>

#endif

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestActionClient;
typedef vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr RequestResult;

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GenerateFeetPoseAction> GenerateFeetPoseActionClient;
typedef vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr GenerateFeetPoseResult;


namespace vigir_footstep_planning_rviz_plugin
{
// Base class for pattern and planning request handlers
// both pattern and planning request handlers can:
//  - send a request for step plan and emit result step plan
//  - ask for feedback and emit it
//  - set general properties of request message
//  - connect to action server and emit when connection has changed
//  - ask for current pose of start feet and emit it
//  - add step plan to existing one by:
//      -> using current goal as start for computing next step plan
//      -> appending next step plan to current and emitting resulting step plan
//  - set current goal either to last step or to chosen step
class RequestHandlerBase : public QObject
{
  Q_OBJECT
public:
  RequestHandlerBase(QObject *parent = 0);
  virtual ~RequestHandlerBase();

  void connectToActionServer();
  bool checkConnection();

  void sendRequest();
  void cancelGoals();

  void requestStartPose();

  void setPlanningMode(int planning_mode);


public Q_SLOTS:
  void setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);

  void setFeedbackRequested(bool requested);
  void setCurrentGoal(int last_index);

  void setStartFoot(int start_foot_selection); //start_foot_selection = RequestMsg::RIGHT or RequestMsg::LEFT
  void setParameterSet(QString parameter_set_name);
  void setFrameID(QString frame_id);

  void resetStepPlan();

Q_SIGNALS:
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan s);
  void createdSequence(vigir_footstep_planning_msgs::StepPlan sequence); // values need to be updated
  void receivedPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void startFeetAnswer(vigir_footstep_planning_msgs::Feet start);
  void stepPlanGenerationStarted();
  void stepPlanGenerationFinished(bool success);

protected:
  void addStepPlan();
  virtual void appendStepPlan(StepPlanMsg add) {};
  void setHeaderStamp();
  void computeShift(float& x_shift, float& y_shift, float& z_shift, const geometry_msgs::Quaternion &orientation);

  vigir_footstep_planning_msgs::StepPlanRequest* request_;

  StepPlanRequestActionClient step_plan_request_ac;
  GenerateFeetPoseActionClient generate_feet_ac;

  vigir_footstep_planning_msgs::StepPlan current_step_plan;

  int last_step_index;
  int replan_goal_index;
private:
  bool feedback_requested_;

  void addStepPlanCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result);
  void setCurrentGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void startPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void resultCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result);
  void feedbackCallback(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);

};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // REQUEST_HANDLER_H
