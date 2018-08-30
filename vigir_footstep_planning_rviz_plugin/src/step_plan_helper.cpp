#include <vigir_footstep_planning_rviz_plugin/step_plan_helper.h>
#include <vigir_footstep_planning_msgs/step_plan.h>
#include <vigir_footstep_planning_rviz_plugin/common/ogre_visualization_msgs_functions.h>

namespace vigir_footstep_planning_rviz_plugin
{

StepPlanHelper::StepPlanHelper(QObject *parent)
  : QObject( parent )
  , edit_step_ac("edit_step", true)
  , update_step_plan_ac("update_step_plan", true)
  , execution_state(-1)
  , last_performed_step(-1)
  , last_action(GENERAL)
  , update_positions(false)
{
  std::string execute_name;
  ros::NodeHandle nh;
  if(nh.getParam("execute_action_server", execute_name))
    execute_step_plan_ac = new ExecuteStepPlanActionClient(execute_name, true);
  else
    ROS_ERROR("Could not find parameter \"execute_action_server\"");

  connectToActionServer();
}

StepPlanHelper::~StepPlanHelper()
{
  delete execute_step_plan_ac;
}

// ----------- Action Server Connection --------------------------------------------
void StepPlanHelper::connectToActionServer()
{
  edit_step_ac.waitForServer(ros::Duration(1,0));
  if(execute_step_plan_ac)
    execute_step_plan_ac->waitForServer(ros::Duration(1,0));
  update_step_plan_ac.waitForServer(ros::Duration(1,0));
}

bool StepPlanHelper::checkConnectionEditStep()
{
  return edit_step_ac.isServerConnected();
}

bool StepPlanHelper::checkConnectionExecute()
{
  if(execute_step_plan_ac)
    return execute_step_plan_ac->isServerConnected();
  else
    return false;
}

bool StepPlanHelper::checkConnectionUpdateStepPlan()
{
  return update_step_plan_ac.isServerConnected();
}

bool StepPlanHelper::executeStepPlanActive()
{
  if(execute_step_plan_ac)
    return execute_step_plan_ac->isServerConnected();

  return false;
}


// ---------- Step Plan Handling ------------------------------------------

// invoked by signal createdStepPlan
void StepPlanHelper::setCurrentStepPlan(StepPlanMsg step_plan)
{
  //is called when new step plan has been newly generated
  previous_sequences.clear(); //not in sequence mode any more
  last_action = GENERAL;
  current_step_plan = step_plan;
  if(update_positions)
  {
    updateStepPlanPositions();
  }
  else
    previous_step_plans.push_back(current_step_plan);
}

// Accept step plan when edited foot has been dropped, update cost
void StepPlanHelper::acceptModifiedStepPlan()
{
  last_action = GENERAL;
  updateStepPlanCost(current_step_plan);
}

void StepPlanHelper::handleSequence(vigir_footstep_planning_msgs::StepPlan sequence)
{
  last_action = ADD_SEQUENCE;
  previous_sequences.push_back(current_step_plan);
  updateStepPlanCost(sequence);
}

// --------- Update Step Plan ---------------------------------------------
void StepPlanHelper::updateStepPlan(vigir_footstep_planning_msgs::UpdateMode update_mode, vigir_footstep_planning_msgs::StepPlan step_plan)
{
  if(update_step_plan_ac.isServerConnected() && step_plan.steps.size() > 0)
  {
    vigir_footstep_planning_msgs::UpdateStepPlanGoal goal;
    goal.step_plan = step_plan;
    goal.update_mode = update_mode;
    update_step_plan_ac.sendGoal(goal,
                                 boost::bind(&StepPlanHelper::updateStepPlanCallback, this, _1, _2),
                                 UpdateStepPlanActionClient::SimpleActiveCallback(),
                                 UpdateStepPlanActionClient::SimpleFeedbackCallback());
  }
  else
  {
    ROS_INFO("update step plan server not connected");
  }
}

void StepPlanHelper::updateStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result)
{
  if(!(result->status.error == ErrorStatusMsg::NO_ERROR) || !state.isDone())
    return;

  if(result->step_plan.steps.size() == current_step_plan.steps.size())
  {
    Q_EMIT(updatedStepPlan(result->step_plan));
  }
  else
  {
    Q_EMIT(createdStepPlan(result->step_plan));
  }
  current_step_plan = result->step_plan;
  previous_step_plans.push_back(result->step_plan);
}

void StepPlanHelper::updateStepPlanPositions()
{
  vigir_footstep_planning_msgs::UpdateMode update_mode;
  update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D;
  updateStepPlan(update_mode, current_step_plan);
}

void StepPlanHelper::updateStepPlanCost(vigir_footstep_planning_msgs::StepPlan step_plan)
{
  //update Step Plan Cost
  vigir_footstep_planning_msgs::UpdateMode update_mode;
  update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_COST;
  updateStepPlan(update_mode, step_plan);
}

void StepPlanHelper::setUpdateStepPlanPositions(bool update)
{
  this->update_positions = update;
}

// --------- Modify Step Plan -------------------------------------------
void StepPlanHelper::trimStepPlan(int last_index)
{
  StepPlanMsg trimmed = current_step_plan;
  trimmed.steps.resize(last_index + 1);
  Q_EMIT(createdStepPlan(trimmed));
  previous_step_plans.push_back(trimmed);
  current_step_plan = trimmed;
  last_action = GENERAL;
}

// ----------- Edit Step -------------------------------------------------
void StepPlanHelper::editStep(vigir_footstep_planning_msgs::EditStep edit_step)
{
  if(edit_step_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::EditStepGoal goal;
    goal.step_plan = current_step_plan;
    goal.edit_step = edit_step;
    edit_step_ac.sendGoal(goal,
                          boost::bind(&StepPlanHelper::editStepCallback, this, _1, _2),
                          EditStepActionClient::SimpleActiveCallback(),
                          EditStepActionClient::SimpleFeedbackCallback());
  }
  else
    ROS_WARN("edit_step not available! Please activate an \"edit_step\" action server.");

}

void StepPlanHelper::editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
  if(!(result->status.error == ErrorStatusMsg::NO_ERROR) || !state.isDone())
    return;

  if(result->step_plans.size() == 1)
  {
    current_step_plan = result->step_plans[0];
  }
  else
  {
    ROS_ERROR("cannot handle deleted step");
    //    std::vector<StepPlanMsg> step_plans = result->step_plans;
    //    combineStepPlans(step_plans);
  }
  checkSteps(); // check steps to display their validity as it is not yet updated
}

void StepPlanHelper::checkSteps()
{
  for (int i = 0; i < current_step_plan.steps.size(); ++i)
  {
    StepMsg step = current_step_plan.steps[i];
    if(!step.valid || step.colliding)
    {
      Q_EMIT(stepValidUpdate(step.step_index, false));
      if(step.colliding)
        ROS_INFO("colliding");
    }
    else
      Q_EMIT(stepValidUpdate(step.step_index, true));
  }
}

// --------- Reset, Undo, ... -------------------------------------------

void StepPlanHelper::resetStepPlan()
{
  current_step_plan = vigir_footstep_planning_msgs::StepPlan();
  previous_step_plans.push_back(current_step_plan);
  previous_sequences.clear();
  last_action = GENERAL;
}

void StepPlanHelper::setPreviousStepPlan()
{
  StepPlanMsg previous;
  bool do_nothing = true;

  if(previous_step_plans.size() > 1)
  {
    do_nothing = false;
    previous_step_plans.pop_back();
    previous = previous_step_plans.back();
  }
  switch(last_action)
  {
  case GENERAL:
    break;
  case ADD_SEQUENCE:
    // if sequence exists and previous plan has a different number of steps it must undo undid adding of sequence:
    if(previous_sequences.size() > 0)
    {
      if(!do_nothing)
      {
        previous_sequences.pop_back(); // remove last added sequence
        last_action = REMOVE_SEQUENCE;
      }
    }
    break;
  case REMOVE_SEQUENCE:
    if(!do_nothing)
    {
      previous_sequences.push_back(previous);
      last_action = ADD_SEQUENCE;
    }
    break;
  }

  if(!do_nothing)
  {
    if(previous.steps.size() == current_step_plan.steps.size())
      Q_EMIT(updatedStepPlan(previous));
    else
    {
      Q_EMIT(createdStepPlan(previous));
    }
    current_step_plan = previous;
  }
}

void StepPlanHelper::setPreviousSequence()
{
  if(previous_sequences.size() > 0)
  {
    StepPlanMsg previous = previous_sequences.back();
    previous_sequences.pop_back();
    current_step_plan = previous;
  }
  else
  {
    current_step_plan.steps.clear();
  }
  last_action = REMOVE_SEQUENCE;
  previous_step_plans.push_back(current_step_plan);
  Q_EMIT(createdStepPlan(current_step_plan));
}


// ----------- Execution -------------------------------------------------
void StepPlanHelper::executeStepPlan()
{
  if(execute_step_plan_ac && execute_step_plan_ac->isServerConnected())
  {
    Q_EMIT(executionStarted(current_step_plan.steps.size()));
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal goal;
    goal.step_plan = current_step_plan;
    execute_step_plan_ac->sendGoal(goal,
                                   boost::bind(&StepPlanHelper::executeStepPlanCallback, this, _1, _2),
                                   ExecuteStepPlanActionClient::SimpleActiveCallback(),
                                   boost::bind(&StepPlanHelper::executeStepPlanFeedbackCallback, this, _1));

  }
  else
    ROS_WARN("execute_step_plan not available! Please activate an \"execute_step_plan\" action server.");

}

void StepPlanHelper::executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
  execution_state = -1;
  switch(result->controller_state)
  {
  case 0: //NOT_READY
    ROS_ERROR("not ready");
    break;
  case 1: // READY
    ROS_ERROR("ready");
    break;
  case 2: // ACTIVE
    ROS_ERROR("active");
    break;
  case 3: // PAUSED
    ROS_ERROR("paused");
    break;
  case 4: // FINISHED
    Q_EMIT(executionFinished(true));
    break;
  case 5: // FAILED
    Q_EMIT(executionFinished(false));
    break;
  }
}

void StepPlanHelper::executeStepPlanFeedbackCallback(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
  if(!(feedback->last_performed_step_index == last_performed_step))
  {
    QString message = QStringLiteral("Completed Step %1").arg(feedback->last_performed_step_index)
        + QStringLiteral(" of %1").arg(current_step_plan.steps.size()-1);

    Q_EMIT(executionFeedback(feedback->last_performed_step_index));
    last_performed_step = feedback->last_performed_step_index;
  }
}

void StepPlanHelper::abortExecution()
{
  ROS_ERROR("abort execution");
  if(execute_step_plan_ac)
    execute_step_plan_ac->cancelAllGoals();
}

/*
void StepPlanHelper::combineStepPlans(std::vector<StepPlanMsg>& step_plans)
{
  if (step_plans.size() != 2)
  {
    ROS_WARN("Function combineStepPlans() only meant for stitching when one step is deleted.");
    return;
  }
  vigir_footstep_planning::StepPlan result(step_plans[0]);
  vigir_footstep_planning_msgs::ErrorStatus error_status;
  ROS_INFO("first step plan size: %i", (int)step_plans[0].steps.size());
  ROS_INFO("second step plan size: %i", (int)step_plans[1].steps.size());
  for(int i = 0; i < step_plans[1].steps.size(); ++i)
  {
    step_plans[1].steps[i].step_index = i;
    ROS_INFO("set index of 2nd plan to %i", (int) step_plans[1].steps[i].step_index);
  }
  error_status = result.appendStepPlan(step_plans[1]);
  if(checkForErrors(error_status))
    return;

  result.toMsg(current_step_plan);
  previous_step_plans.push_back(current_step_plan);
  ROS_INFO("size prev step_plan: %i", (int)previous_step_plans.size());
  Q_EMIT(createdStepPlan(current_step_plan));
}
*/

/*
void StepPlanHelper::addStep(const std::string& frame_id, const Ogre::Vector3& position, const Ogre::Quaternion& orientation, unsigned int which_foot, unsigned int step_index)
{
  //todo: EditStep benutzen?
  StepMsg step;
  geometry_msgs::Pose pose;
  pose.position.x = position.x; pose.position.y = position.y; pose.position.z = position.z;
  pose.orientation.w = orientation.w; pose.orientation.x = orientation.x; pose.orientation.y = orientation.y; pose.orientation.z = orientation.z;
  step.step_index = step_index;
  step.foot.foot_index = which_foot;
  step.foot.pose = pose;
  step.header.frame_id = frame_id;
  step.header.stamp = ros::Time::now();
  step.foot.header = step.header;

  vigir_footstep_planning::StepPlan current(current_step_plan);
  current.insertStep(step);
  current.toMsg(current_step_plan);
}*/


} // end namespace vigir_footstep_planning_rviz_plugin



