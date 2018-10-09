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


#ifndef STEP_PLAN_HELPER_H
#define STEP_PLAN_HELPER_H

#ifndef Q_MOC_RUN

#include <QObject>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <actionlib/client/simple_action_client.h>


#endif

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction> ExecuteStepPlanActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateStepPlanAction> UpdateStepPlanActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateFootAction> UpdateFootActionClient;


namespace vigir_footstep_planning_rviz_plugin
{
/* modify step plan
 * update step plan
 * modify steps in step plan
 *
 * check state of step plan
 * check state of steps
 *
*/

enum LastActionType {GENERAL = 0, ADD_SEQUENCE = 1, REMOVE_SEQUENCE = 2};

class StepPlanHelper : public QObject
{
  Q_OBJECT
public:
  StepPlanHelper(QObject *parent = 0);
  virtual ~StepPlanHelper();

  // Acion Server Connection
  void connectToActionServer();
  bool checkConnectionEditStep();
  bool checkConnectionExecute();
  bool checkConnectionUpdateStepPlan();


  // checks if steps of current step plan are valid, emits stepValidUpdate
  void checkSteps();
  // sets if positions should be updated when a new step plan is set
  void setUpdateStepPlanPositions(bool update);

  bool executeStepPlanActive();

public Q_SLOTS:
  // called with edited step, sends goal to edit_step action client, result is handled in editStepCallback
  void editStep(vigir_footstep_planning_msgs::EditStep edit_step);
  // called when executing is requested, sends goal to execute_step_plan action client, result is handled in executStepPlanCallback
  void executeStepPlan();
  // update current step plan, update positions if requested
  void setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // invoked from clicking undo button set to previous step plan in list previous_step_plans
  void setPreviousStepPlan();
  // remove last added sequence part
  void setPreviousSequence();
  // called after interaction with foot, when foot is dropped, to update current step plan
  void acceptModifiedStepPlan();
  // The step plan positions will be updated to fit the current terrain
  void updateStepPlanPositions();
  // Called to update the cost after step plan has been modified [TODO]
  void updateStepPlanCost(vigir_footstep_planning_msgs::StepPlan step_plan);
  // Step Plan is reset to empty message
  void resetStepPlan();
  // Step Plan will be cut at last_index, last_index is the index of the step which will be last
  void trimStepPlan(int last_index);
  // Handle a generated sequence by updating cost
  void handleSequence(vigir_footstep_planning_msgs::StepPlan sequence);
  void abortExecution();

Q_SIGNALS:
  // emitted from checkSteps() to update step visuals to display their validity
  void stepValidUpdate(unsigned int step_index, bool valid);
  // if a step plan has been modified such that the number of step have changed (trimStepPlan, deleted step, set previous)
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // if a step plan has been modified without changing the number of steps the positions etc will only be updated (update functions, setPrevious, acceptModified)
  void updatedStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // signal to communicate that execution has started, this will invoke initialization of the progress bar
  void executionStarted(int nofSteps);
  void executionFinished(bool success);
  // signal with execution feedback emitting last finished step, this is displayed in the progress bar
  void executionFeedback(int last_performed);


private:
  // Action Client Callbacks:
  void editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);
  void executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);
  void executeStepPlanFeedbackCallback(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
  void updateStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result);
  // called to start update_step_plan action client with the wanted update mode (called by updateStepPlanPositions, updateStepPlanCost)
  void updateStepPlan(vigir_footstep_planning_msgs::UpdateMode update_mode, vigir_footstep_planning_msgs::StepPlan step_plan);


  // the currently displayed step plan
  StepPlanMsg current_step_plan;

  // Action Clients:
  EditStepActionClient edit_step_ac;
  UpdateStepPlanActionClient update_step_plan_ac;
  ExecuteStepPlanActionClient* execute_step_plan_ac;

  bool update_positions;

// For Execution Status:
  int execution_state;
  int last_performed_step;
  /*
   * Possible Actions:
   * - Step Plan was created -> GENERAL
   * - Step was edited -> GENERAL
   * - Part of Sequence was added -> ADD_SEQUENCE
   * - Last Part of Sequence was removed -> REMOVE_SEQUENCE
   * */
  LastActionType last_action;

  //stores the last step plans, previous_step_plans.back() = current_step_plan
  std::vector<vigir_footstep_planning_msgs::StepPlan> previous_step_plans;
  std::vector<vigir_footstep_planning_msgs::StepPlan> previous_sequences;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_PLAN_HELPER_H
